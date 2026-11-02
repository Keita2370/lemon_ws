// pregrasp からLemonを掴んで pregrasp へ戻るまでを MoveIt Servo (PoseTracking) で行う。
//
// ~/grasp (Trigger):
//   グリッパを開く
//   → 姿勢は開始時 (pregrasp) のまま固定し、位置だけ /lemon_target/tracked に追従して
//     指先をLemonへ入れる (検出が途切れたら最後の位置のまま進む)
//   → グリッパを閉じ、閉じきらなければ把持成功としてLemonをグリッパに取り付ける
//   → 開始時の姿勢へ戻る (行きの逆。把持していればここで引き抜く)。
//     retreat_mode が cartesian なら直線のデカルト軌道を arm_controller の
//     アクションでまとめて実行する (Servo の揺れが乗らず速い)。作れなければ Servo で戻る
// ~/open (Trigger): グリッパを開き、取り付けたLemonを外す
//
// 把持中以外は Servo を pause し、move_group の軌道実行と指令が重ならないようにする。

#include <atomic>
#include <chrono>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/cartesian_interpolator.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <moveit_servo/pose_tracking.h>
#include <moveit_servo/servo_parameters.h>
#include <moveit_servo/status_codes.h>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

using namespace std::chrono_literals;

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using Trigger = std_srvs::srv::Trigger;
using moveit_servo::PoseTrackingStatusCode;

namespace
{

int axis_index(const std::string & name)
{
  if (name == "x") {return 0;}
  if (name == "y") {return 1;}
  if (name == "z") {return 2;}
  throw std::invalid_argument("axis must be x, y or z: " + name);
}

template<typename FutureT>
bool wait_future(FutureT & future, double timeout)
{
  return future.wait_for(std::chrono::duration<double>(timeout)) == std::future_status::ready;
}

}  // namespace

class LemonGrasp : public rclcpp::Node
{
public:
  LemonGrasp()
  : Node("lemon_grasp")
  {
    // Lemonへ向ける ee_link の軸と、ee_link から指先までの距離 [m]
    // (lemon_approach_node と同じ値にする)
    approach_axis_ = axis_index(declare_parameter("approach_axis", "z"));
    tcp_offset_ = declare_parameter("tcp_offset", 0.136);
    // 指先をLemon中心からさらに approach 方向へ入れる距離 [m]
    grasp_depth_ = declare_parameter("grasp_depth", 0.0);

    position_tolerance_ = declare_parameter("position_tolerance", 0.005);
    angular_tolerance_ = declare_parameter("angular_tolerance", 0.05);
    // 1回の Servo 移動 (接近 / 戻り) の制限時間 [s]
    servo_timeout_ = declare_parameter("servo_timeout", 20.0);
    // PoseTracking がこの時間 [s] 目標を受け取らないと中断する
    target_pose_timeout_ = declare_parameter("target_pose_timeout", 0.5);
    target_rate_ = declare_parameter("target_rate", 50.0);
    // tracked がこの時間 [s] 更新されなければ見失ったとみなす (最後の位置のまま進む)
    track_timeout_ = declare_parameter("track_timeout", 0.5);
    // 目標にするLemon位置の移動速度の上限 [m/s]。検出の飛びでアームが振られないようにする
    max_lemon_speed_ = declare_parameter("max_lemon_speed", 0.05);
    // 指先がLemon中心のこの距離 [m] 手前まで来たら、Lemon位置の更新をやめて目標を固定する。
    // 近づくとLemonが指やグリッパの影に隠れて検出が不安定になり、まばらな検出で
    // 目標が隣のLemonへずれていくため
    freeze_distance_ = declare_parameter("freeze_distance", 0.04);
    // PoseTracking に渡す目標を手先からこの距離 [m] までに制限する。
    // 指令速度は最大で (位置のPゲイン) * この距離 になる
    max_target_distance_ = declare_parameter("max_target_distance", 0.03);

    gripper_action_ = declare_parameter(
      "gripper_action", "/gripper_controller/follow_joint_trajectory");
    gripper_joint_ = declare_parameter("gripper_joint", "joint7");
    gripper_open_ = declare_parameter("gripper_open", 0.035);
    gripper_close_ = declare_parameter("gripper_close", 0.0);
    gripper_duration_ = declare_parameter("gripper_duration", 1.0);
    // 閉じた後の gripper_joint がこれより大きければLemonを挟んでいるとみなす
    min_grasp_width_ = declare_parameter("min_grasp_width", 0.005);
    // 閉じた後、gripper_joint がこの時間 [s] 動かなくなったら判定する
    gripper_settle_time_ = declare_parameter("gripper_settle_time", 0.5);

    // 把持したLemonを取り付けるリンクと、衝突を許すリンク
    ee_link_ = declare_parameter("ee_link", "gripper_base");
    touch_links_ = declare_parameter(
      "touch_links", std::vector<std::string>{"gripper_base", "link7", "link8"});
    lemon_radius_ = declare_parameter("lemon_radius", 0.03);
    lemon_object_id_ = declare_parameter("lemon_object_id", "grasped_lemon");

    service_timeout_ = declare_parameter("service_timeout", 5.0);
    // Servo の出力 (~/servo_trajectory) を時刻をずらして送る先
    trajectory_topic_ = declare_parameter("trajectory_topic", "/arm_controller/joint_trajectory");
    // Servo の軌道の最初の点を 0 秒にずらすか (Isaac Sim の 25 Hz の /clock 用)。
    // 実機では JTC が publish_period かけて補間できるよう false にする
    // (true だと publish_period ごとの階段状の指令になって揺れる)
    shift_servo_trajectory_ = declare_parameter("shift_servo_trajectory", true);

    // 戻り方: "cartesian" (直線軌道をまとめて実行) / "servo"
    retreat_mode_ = declare_parameter("retreat_mode", "cartesian");
    arm_action_ = declare_parameter(
      "arm_action", "/arm_controller/follow_joint_trajectory");
    // cartesian の戻りでの各関節の速度 [rad/s] と加速度 [rad/s^2] の上限
    retreat_max_velocity_ = declare_parameter("retreat_max_velocity", 0.5);
    retreat_max_acceleration_ = declare_parameter("retreat_max_acceleration", 1.0);

    cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    status_pub_ = create_publisher<std_msgs::msg::String>("~/status", 10);
    // PoseTracking が購読する topic
    target_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "target_pose", rclcpp::SystemDefaultsQoS());
    trajectory_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      trajectory_topic_, rclcpp::SystemDefaultsQoS());

    rclcpp::SubscriptionOptions options;
    options.callback_group = cb_group_;
    tracked_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "/lemon_target/tracked", rclcpp::QoS(1).transient_local(),
      [this](geometry_msgs::msg::PointStamped::ConstSharedPtr msg) {tracked_callback(msg);},
      options);
    servo_trajectory_sub_ = create_subscription<trajectory_msgs::msg::JointTrajectory>(
      "~/servo_trajectory", rclcpp::SystemDefaultsQoS(),
      [this](trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
        servo_trajectory_callback(msg);
      },
      options);

    gripper_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
      this, gripper_action_, cb_group_);
    arm_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
      this, arm_action_, cb_group_);
    apply_scene_client_ = create_client<moveit_msgs::srv::ApplyPlanningScene>(
      "/apply_planning_scene", rmw_qos_profile_services_default, cb_group_);

    grasp_srv_ = create_service<Trigger>(
      "~/grasp",
      [this](Trigger::Request::SharedPtr, Trigger::Response::SharedPtr res) {grasp_callback(res);},
      rmw_qos_profile_services_default, cb_group_);
    open_srv_ = create_service<Trigger>(
      "~/open",
      [this](Trigger::Request::SharedPtr, Trigger::Response::SharedPtr res) {open_callback(res);},
      rmw_qos_profile_services_default, cb_group_);

    target_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / target_rate_),
      [this]() {publish_target();}, cb_group_);
  }

  // PoseTracking は shared_from_this() が必要で、joint_states の受信も待つので、
  // executor を回し始めてから呼ぶ
  bool init()
  {
    auto node = shared_from_this();

    servo_params_ = moveit_servo::ServoParameters::makeServoParameters(node);

    if (!servo_params_) {
      RCLCPP_FATAL(get_logger(), "Could not get servo parameters");
      return false;
    }

    planning_frame_ = servo_params_->planning_frame;

    psm_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
      node, "robot_description");

    if (!psm_->getPlanningScene()) {
      RCLCPP_FATAL(get_logger(), "Failed to set up the planning scene monitor");
      return false;
    }

    // シーン (octomap) は監視しない。最終接近ではLemonや枝のボクセルへ自分から
    // 入りに行くので、Servo の衝突チェックは自己干渉だけにする
    psm_->startStateMonitor(servo_params_->joint_topic);

    std::vector<std::string> missing;

    while (rclcpp::ok() && !psm_->getStateMonitor()->haveCompleteState(missing)) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000, "Waiting for joint states (%zu missing)",
        missing.size());
      missing.clear();
      std::this_thread::sleep_for(100ms);
    }

    tracker_ = std::make_unique<moveit_servo::PoseTracking>(node, servo_params_, psm_);
    tracker_->servo_->setPaused(true);

    servo_status_sub_ = create_subscription<std_msgs::msg::Int8>(
      servo_params_->status_topic, 10,
      [this](std_msgs::msg::Int8::ConstSharedPtr msg) {servo_status_callback(msg);});

    ready_ = true;
    RCLCPP_INFO(get_logger(), "Ready");

    return true;
  }

private:
  enum class Mode { IDLE, TRACK, RETREAT };

  // --------------------------------------------------
  // Callbacks
  // --------------------------------------------------

  void tracked_callback(geometry_msgs::msg::PointStamped::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    // frame_idが空なら lemon_target 側で対象が解除されている
    if (msg->header.frame_id.empty()) {
      lemon_.reset();
      return;
    }

    lemon_ = *msg;
    lemon_received_ = std::chrono::steady_clock::now();
  }

  // Servo は最初の点を publish_period 後に置くが、JTC は受け取った時点の状態から
  // そこへ補間するので、sim の /clock が進む前に次の軌道で置き換わると指令が
  // ほとんど進まない (Isaac Sim は 40 ms 刻み)。最初の点を 0 秒にずらして、
  // 受け取ってすぐその位置を指令させる
  void servo_trajectory_callback(trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
  {
    if (msg->points.empty()) {
      return;
    }

    if (!shift_servo_trajectory_) {
      trajectory_pub_->publish(*msg);
      return;
    }

    const rclcpp::Duration shift(msg->points.front().time_from_start);

    for (auto & point : msg->points) {
      point.time_from_start = rclcpp::Duration(point.time_from_start) - shift;
    }

    trajectory_pub_->publish(*msg);
  }

  void servo_status_callback(std_msgs::msg::Int8::ConstSharedPtr msg)
  {
    auto status = static_cast<moveit_servo::StatusCode>(msg->data);

    if (status == servo_status_) {
      return;
    }

    servo_status_ = status;
    RCLCPP_INFO(
      get_logger(), "Servo status: %s",
      moveit_servo::SERVO_STATUS_CODE_MAP.at(status).c_str());
  }

  void grasp_callback(Trigger::Response::SharedPtr response)
  {
    if (!ready_) {
      response->success = false;
      response->message = "not ready";
      return;
    }

    if (busy_.exchange(true)) {
      response->success = false;
      response->message = "busy";
      return;
    }

    bool has_target;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      has_target = lemon_.has_value();
    }

    if (!has_target) {
      busy_ = false;
      response->success = false;
      response->message = "no target";
      return;
    }

    std::tie(response->success, response->message) = grasp();
    busy_ = false;

    publish_status(response->message);

    if (response->success) {
      RCLCPP_INFO(get_logger(), "Grasp succeeded");
    } else {
      RCLCPP_ERROR(get_logger(), "Grasp failed: %s", response->message.c_str());
    }
  }

  void open_callback(Trigger::Response::SharedPtr response)
  {
    if (busy_.exchange(true)) {
      response->success = false;
      response->message = "busy";
      return;
    }

    response->success = move_gripper(gripper_open_);
    response->message = response->success ? "opened" : "failed to open gripper";

    // 開けば手を離れるので、開けなかった場合も取り付けは外す
    detach_lemon();
    busy_ = false;
  }

  // --------------------------------------------------
  // 把持
  // --------------------------------------------------

  std::pair<bool, std::string> grasp()
  {
    publish_status("opening");

    if (!move_gripper(gripper_open_)) {
      return {false, "failed to open gripper"};
    }

    geometry_msgs::msg::TransformStamped ee;

    if (!tracker_->getCommandFrameTransform(ee)) {
      return {false, "failed to get end effector pose"};
    }

    const Eigen::Isometry3d start = tf2::transformToEigen(ee);

    {
      std::lock_guard<std::mutex> lock(mutex_);
      // 姿勢は pregrasp のまま固定し、位置だけLemonに追従させる
      orientation_ = Eigen::Quaterniond(start.rotation());
      approach_ = start.rotation().col(approach_axis_);
      retreat_goal_ = start.translation();
    }

    publish_status("inserting");
    auto [inserted, insert_message] = servo_to(Mode::TRACK);

    bool grasped = false;
    std::string message;

    if (inserted) {
      publish_status("closing");

      // Lemonを挟むと目標まで閉じきらずにabortされるので、結果は開き具合で判断する。
      // アクションは軌道の時間が過ぎると位置を見ずに終わるので、止まるまで待つ
      move_gripper(gripper_close_);
      wait_gripper_settled();

      const double width = gripper_position();
      grasped = width > min_grasp_width_;

      if (grasped) {
        attach_lemon();
        message = "grasped";
      } else {
        message = "missed (gripper closed to " + std::to_string(width) + ")";
      }

    } else {
      message = "insertion failed: " + insert_message;
    }

    // 行きの逆: 接近を始めた pregrasp まで戻る (把持していればここで引き抜く)。
    // 失敗時も戻しておかないと、Lemonや枝に近すぎて home への計画が通らない
    publish_status("retreating");
    auto [retreated, retreat_message] = retreat();

    if (!retreated) {
      return {false, message + ", retreat failed: " + retreat_message};
    }

    return {grasped, message};
  }

  std::pair<bool, std::string> retreat()
  {
    if (retreat_mode_ == "cartesian") {
      auto [ok, message] = retreat_cartesian();

      if (ok) {
        return {true, message};
      }

      RCLCPP_WARN(
        get_logger(), "Cartesian retreat failed (%s), retreating with Servo", message.c_str());
    }

    return servo_to(Mode::RETREAT);
  }

  // 今の手先から retreat_goal_ まで、姿勢を保ったまま直線で戻る軌道を作り、
  // arm_controller にまとめて実行させる
  std::pair<bool, std::string> retreat_cartesian()
  {
    auto state = psm_->getStateMonitor()->getCurrentState();
    const auto * group = state->getJointModelGroup(servo_params_->move_group_name);
    const auto * link = state->getLinkModel(servo_params_->robot_link_command_frame);

    if (!group || !link) {
      return {false, "unknown group or link"};
    }

    Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      target.linear() = orientation_.toRotationMatrix();
      target.translation() = retreat_goal_;
    }

    planning_scene_monitor::LockedPlanningSceneRO scene(psm_);
    auto valid = [&scene](
      moveit::core::RobotState * s, const moveit::core::JointModelGroup * g, const double * values)
      {
        s->setJointGroupPositions(g, values);
        s->update();
        return !scene->isStateColliding(*s, g->getName());
      };

    std::vector<moveit::core::RobotStatePtr> path;
    const double fraction = moveit::core::CartesianInterpolator::computeCartesianPath(
      state.get(), group, path, link, target, true,
      moveit::core::MaxEEFStep(0.005), moveit::core::JumpThreshold(0.3, 0.0), valid);

    if (fraction < 0.99) {
      return {false, "cartesian path is only " + std::to_string(fraction * 100.0) + "% feasible"};
    }

    robot_trajectory::RobotTrajectory trajectory(state->getRobotModel(), group);

    for (const auto & point : path) {
      trajectory.addSuffixWayPoint(point, 0.0);
    }

    std::unordered_map<std::string, double> velocity_limits;
    std::unordered_map<std::string, double> acceleration_limits;

    for (const auto & name : group->getActiveJointModelNames()) {
      velocity_limits[name] = retreat_max_velocity_;
      acceleration_limits[name] = retreat_max_acceleration_;
    }

    trajectory_processing::TimeOptimalTrajectoryGeneration totg;

    if (!totg.computeTimeStamps(trajectory, velocity_limits, acceleration_limits)) {
      return {false, "time parameterization failed"};
    }

    moveit_msgs::msg::RobotTrajectory msg;
    trajectory.getRobotTrajectoryMsg(msg);

    RCLCPP_INFO(
      get_logger(), "Cartesian retreat: %zu points, %.2f s",
      msg.joint_trajectory.points.size(), trajectory.getDuration());

    return execute_arm(msg.joint_trajectory, trajectory.getDuration());
  }

  std::pair<bool, std::string> execute_arm(
    const trajectory_msgs::msg::JointTrajectory & trajectory, double duration)
  {
    if (!arm_client_->wait_for_action_server(std::chrono::duration<double>(service_timeout_))) {
      return {false, arm_action_ + " not available"};
    }

    FollowJointTrajectory::Goal goal;
    goal.trajectory = trajectory;

    auto goal_future = arm_client_->async_send_goal(goal);

    if (!wait_future(goal_future, service_timeout_)) {
      return {false, "arm goal timed out"};
    }

    auto handle = goal_future.get();

    if (!handle) {
      return {false, "arm goal rejected"};
    }

    auto result_future = arm_client_->async_get_result(handle);

    if (!wait_future(result_future, duration + service_timeout_)) {
      arm_client_->async_cancel_goal(handle);
      return {false, "arm result timed out"};
    }

    const auto result = result_future.get();

    if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
      return {false, "arm trajectory failed: " + result.result->error_string};
    }

    return {true, "reached"};
  }

  // mode の目標へ Servo で移動する (目標は publish_target が配信する)
  std::pair<bool, std::string> servo_to(Mode mode)
  {
    tracker_->resetTargetPose();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      goal_lemon_.reset();
      goal_frozen_ = false;
    }
    timed_out_ = false;
    deadline_ = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(servo_timeout_));

    tracker_->servo_->setPaused(false);
    mode_ = mode;

    const auto code = tracker_->moveToPose(
      Eigen::Vector3d::Constant(position_tolerance_), angular_tolerance_,
      target_pose_timeout_);

    mode_ = Mode::IDLE;
    tracker_->servo_->setPaused(true);

    if (code == PoseTrackingStatusCode::SUCCESS) {
      return {true, "reached"};
    }

    if (timed_out_) {
      return {false, "timeout"};
    }

    return {false, moveit_servo::POSE_TRACKING_STATUS_CODE_MAP.at(code)};
  }

  void publish_target()
  {
    const Mode mode = mode_;

    if (mode == Mode::IDLE) {
      return;
    }

    if (std::chrono::steady_clock::now() > deadline_) {
      if (!timed_out_.exchange(true)) {
        tracker_->stopMotion();
      }
      return;
    }

    Eigen::Vector3d ee_goal;
    Eigen::Quaterniond orientation;

    // 今の手先 (command frame) の位置
    geometry_msgs::msg::TransformStamped ee;
    std::optional<Eigen::Vector3d> current;

    if (tracker_->getCommandFrameTransform(ee)) {
      current = tf2::transformToEigen(ee).translation();
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);

      if (mode == Mode::TRACK && goal_frozen_) {
        ee_goal = *goal_lemon_ + (grasp_depth_ - tcp_offset_) * approach_;

      } else if (mode == Mode::TRACK) {
        // 対象が解除されたら目標を出さない (PoseTracking が中断する)
        if (!lemon_) {
          return;
        }

        if (std::chrono::steady_clock::now() - lemon_received_ >
          std::chrono::duration<double>(track_timeout_))
        {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Lemon is not tracked, continuing to the last position");
        }

        // 見失って時間が経っても変換できるよう、最新のTFを使う
        geometry_msgs::msg::PointStamped lemon = *lemon_;
        lemon.header.stamp = rclcpp::Time(0);

        try {
          lemon = tf_buffer_->transform(lemon, planning_frame_);
        } catch (const tf2::TransformException & e) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "TF failed: %s", e.what());
          return;
        }

        const Eigen::Vector3d measured(lemon.point.x, lemon.point.y, lemon.point.z);

        if (!goal_lemon_) {
          goal_lemon_ = measured;
        } else {
          Eigen::Vector3d step = measured - *goal_lemon_;
          const double max_step = max_lemon_speed_ / target_rate_;

          if (step.norm() > max_step) {
            step *= max_step / step.norm();
          }

          *goal_lemon_ += step;
        }

        const Eigen::Vector3d tcp = *goal_lemon_ + grasp_depth_ * approach_;
        ee_goal = tcp - tcp_offset_ * approach_;

        // 指先がLemon中心の freeze_distance 手前まで来たら目標を固定する
        if (current) {
          const Eigen::Vector3d tip = *current + tcp_offset_ * approach_;

          if ((*goal_lemon_ - tip).dot(approach_) < freeze_distance_) {
            goal_frozen_ = true;
            RCLCPP_INFO(get_logger(), "Close to the lemon; goal is fixed from here");
          }
        }

      } else {
        ee_goal = retreat_goal_;
      }

      orientation = orientation_;
    }

    // 遠い目標は手先から max_target_distance の点に置き換える。目標に着くまでは
    // 誤差が許容値より大きいままなので、途中で到達とはみなされない
    if (current) {
      const Eigen::Vector3d offset = ee_goal - *current;

      if (offset.norm() > max_target_distance_) {
        ee_goal = *current + offset * (max_target_distance_ / offset.norm());
      }
    }

    geometry_msgs::msg::PoseStamped msg;
    msg.header.frame_id = planning_frame_;
    msg.header.stamp = now();
    msg.pose.position.x = ee_goal.x();
    msg.pose.position.y = ee_goal.y();
    msg.pose.position.z = ee_goal.z();
    msg.pose.orientation = tf2::toMsg(orientation);

    target_pose_pub_->publish(msg);
  }

  // --------------------------------------------------
  // グリッパ
  // --------------------------------------------------

  bool move_gripper(double position)
  {
    if (!gripper_client_->wait_for_action_server(std::chrono::duration<double>(service_timeout_))) {
      RCLCPP_ERROR(get_logger(), "%s not available", gripper_action_.c_str());
      return false;
    }

    FollowJointTrajectory::Goal goal;
    goal.trajectory.joint_names = {gripper_joint_};

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {position};
    point.time_from_start = rclcpp::Duration::from_seconds(gripper_duration_);
    goal.trajectory.points.push_back(point);

    auto goal_future = gripper_client_->async_send_goal(goal);

    if (!wait_future(goal_future, service_timeout_)) {
      RCLCPP_ERROR(get_logger(), "Gripper goal timed out");
      return false;
    }

    auto handle = goal_future.get();

    if (!handle) {
      RCLCPP_ERROR(get_logger(), "Gripper goal rejected");
      return false;
    }

    auto result_future = gripper_client_->async_get_result(handle);

    if (!wait_future(result_future, gripper_duration_ + service_timeout_)) {
      RCLCPP_WARN(get_logger(), "Gripper result timed out");
      gripper_client_->async_cancel_goal(handle);
      return false;
    }

    return result_future.get().code == rclcpp_action::ResultCode::SUCCEEDED;
  }

  double gripper_position()
  {
    return psm_->getStateMonitor()->getCurrentState()->getVariablePosition(gripper_joint_);
  }

  // gripper_joint が gripper_settle_time [s] の間ほぼ動かなくなるまで待つ
  void wait_gripper_settled()
  {
    const auto period = std::chrono::duration<double>(gripper_settle_time_);
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(service_timeout_);

    double last = gripper_position();

    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(period);

      const double position = gripper_position();

      if (std::abs(position - last) < 1e-4) {
        return;
      }

      last = position;
    }

    RCLCPP_WARN(get_logger(), "Gripper did not settle");
  }

  // --------------------------------------------------
  // 把持したLemonの取り付け
  // --------------------------------------------------

  // グリッパに取り付けた物体は move_group の octomap から除外されるので、
  // 持ったLemonが映り込んで home への計画の開始姿勢が衝突扱いになるのを防げる
  void attach_lemon()
  {
    moveit_msgs::msg::AttachedCollisionObject aco;
    aco.link_name = ee_link_;
    aco.touch_links = touch_links_;
    aco.object.id = lemon_object_id_;
    aco.object.header.frame_id = ee_link_;
    aco.object.operation = moveit_msgs::msg::CollisionObject::ADD;

    shape_msgs::msg::SolidPrimitive sphere;
    sphere.type = shape_msgs::msg::SolidPrimitive::SPHERE;
    sphere.dimensions = {lemon_radius_};
    aco.object.primitives.push_back(sphere);

    // 指先は grasp_depth だけレモン中心の奥にあるので、球はその手前に置く
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    center[approach_axis_] = tcp_offset_ - grasp_depth_;

    geometry_msgs::msg::Pose pose;
    pose.position = tf2::toMsg(center);
    pose.orientation.w = 1.0;
    aco.object.primitive_poses.push_back(pose);

    moveit_msgs::msg::PlanningScene scene;
    scene.is_diff = true;
    scene.robot_state.is_diff = true;
    scene.robot_state.attached_collision_objects.push_back(aco);

    attached_ = apply_scene(scene);

    if (!attached_) {
      RCLCPP_WARN(get_logger(), "Failed to attach the grasped lemon");
    }
  }

  void detach_lemon()
  {
    if (!attached_) {
      return;
    }

    moveit_msgs::msg::AttachedCollisionObject aco;
    aco.link_name = ee_link_;
    aco.object.id = lemon_object_id_;
    aco.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;

    moveit_msgs::msg::PlanningScene detach;
    detach.is_diff = true;
    detach.robot_state.is_diff = true;
    detach.robot_state.attached_collision_objects.push_back(aco);

    // 取り外した物体は world に残るので、それも消す
    moveit_msgs::msg::CollisionObject object;
    object.id = lemon_object_id_;
    object.header.frame_id = planning_frame_;
    object.operation = moveit_msgs::msg::CollisionObject::REMOVE;

    moveit_msgs::msg::PlanningScene remove;
    remove.is_diff = true;
    remove.world.collision_objects.push_back(object);

    if (!apply_scene(detach) || !apply_scene(remove)) {
      RCLCPP_WARN(get_logger(), "Failed to detach the grasped lemon");
    }

    attached_ = false;
  }

  bool apply_scene(const moveit_msgs::msg::PlanningScene & scene)
  {
    if (!apply_scene_client_->wait_for_service(std::chrono::duration<double>(service_timeout_))) {
      RCLCPP_ERROR(get_logger(), "/apply_planning_scene not available");
      return false;
    }

    auto request = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
    request->scene = scene;

    auto future = apply_scene_client_->async_send_request(request);

    if (!wait_future(future, service_timeout_)) {
      apply_scene_client_->remove_pending_request(future);
      return false;
    }

    return future.get()->success;
  }

  void publish_status(const std::string & text)
  {
    std_msgs::msg::String msg;
    msg.data = text;
    status_pub_->publish(msg);
  }

  // --------------------------------------------------

  int approach_axis_;
  double tcp_offset_;
  double grasp_depth_;
  double position_tolerance_;
  double angular_tolerance_;
  double servo_timeout_;
  double target_pose_timeout_;
  double target_rate_;
  double track_timeout_;
  double max_lemon_speed_;
  double max_target_distance_;
  double freeze_distance_;
  std::string gripper_action_;
  std::string gripper_joint_;
  double gripper_open_;
  double gripper_close_;
  double gripper_duration_;
  double min_grasp_width_;
  double gripper_settle_time_;
  std::string ee_link_;
  std::vector<std::string> touch_links_;
  double lemon_radius_;
  std::string lemon_object_id_;
  double service_timeout_;
  std::string trajectory_topic_;
  bool shift_servo_trajectory_;
  std::string retreat_mode_;
  std::string arm_action_;
  double retreat_max_velocity_;
  double retreat_max_acceleration_;

  rclcpp::CallbackGroup::SharedPtr cb_group_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr tracked_sub_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr servo_trajectory_sub_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr servo_status_sub_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr gripper_client_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr arm_client_;
  rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr apply_scene_client_;
  rclcpp::Service<Trigger>::SharedPtr grasp_srv_;
  rclcpp::Service<Trigger>::SharedPtr open_srv_;
  rclcpp::TimerBase::SharedPtr target_timer_;

  moveit_servo::ServoParameters::SharedConstPtr servo_params_;
  planning_scene_monitor::PlanningSceneMonitorPtr psm_;
  std::unique_ptr<moveit_servo::PoseTracking> tracker_;
  std::string planning_frame_;
  moveit_servo::StatusCode servo_status_ = moveit_servo::StatusCode::INVALID;

  std::atomic<bool> ready_{false};
  std::atomic<bool> busy_{false};
  std::atomic<Mode> mode_{Mode::IDLE};
  std::atomic<bool> timed_out_{false};
  std::chrono::steady_clock::time_point deadline_;
  bool attached_ = false;

  // mutex_ で保護
  std::mutex mutex_;
  std::optional<geometry_msgs::msg::PointStamped> lemon_;
  std::chrono::steady_clock::time_point lemon_received_;
  std::optional<Eigen::Vector3d> goal_lemon_;  // 速度制限をかけた目標のLemon位置
  bool goal_frozen_ = false;                   // 近づいたので goal_lemon_ を固定した
  Eigen::Quaterniond orientation_ = Eigen::Quaterniond::Identity();
  Eigen::Vector3d approach_ = Eigen::Vector3d::UnitZ();
  Eigen::Vector3d retreat_goal_ = Eigen::Vector3d::Zero();
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<LemonGrasp>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() {executor.spin();});

  if (!node->init()) {
    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 1;
  }

  spinner.join();
  rclcpp::shutdown();

  return 0;
}
