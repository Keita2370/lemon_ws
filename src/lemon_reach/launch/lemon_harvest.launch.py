import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


# 実機 (piper_real) では Isaac Sim 用の値を上書きする
REAL_TARGET_PARAMS = {
    # 実機の score は sim よりばらつくので安定判定を少し緩める
    "min_detection_ratio": 0.8,
}
REAL_GRASP_PARAMS = {
    # 接近の指令速度の上限 = Pゲイン (servo_real.yaml) * max_target_distance
    # = 4.0 * 0.02 = 0.08 m/s
    "max_target_distance": 0.02,
    "max_lemon_speed": 0.01,
    "servo_timeout": 40.0,
    # 実機は wall time なので、Servo の軌道は JTC にそのまま補間させる
    "shift_servo_trajectory": False,
    # 戻りは直線のデカルト軌道 (Servo の揺れが乗らない)
    "retreat_mode": "cartesian",
    "retreat_max_velocity": 0.5,
    "retreat_max_acceleration": 1.0,
}
# move_group で動かす区間 (home ⇔ pregrasp) の速さ。joint_limits.yaml は
# 速度 5 rad/s・加速度の制限なし (既定 1 rad/s^2) なので、
# 0.2 → 1.0 rad/s, 0.3 → 0.3 rad/s^2 (sim の既定は 0.1 / 0.1)
REAL_MOVEIT_PARAMS = {
    "velocity_scaling": 0.2,
    "acceleration_scaling": 0.3,
}


def generate_launch_description():
    """
    収穫に使うノードを起動する。YOLO は含まないので、先に別途起動しておく
    (例: ros2 launch lemon_reach lemon_yolo.launch.py)。

    sim:=true で Isaac Sim、false (既定) で実機 (piper_real) 用の設定にする。
    auto_run:=false にすると lemon_harvest_manager を起動しないので、各ノードの
    Service を手で呼んで1段ずつ確かめられる。
    """

    def make_nodes(context):

        def arg(name):
            return LaunchConfiguration(name).perform(context)

        sim = arg("sim").lower() == "true"

        params = [{"use_sim_time": sim}]
        target_params = [{
            "use_sim_time": sim,
            "score_threshold": float(arg("score_threshold")),
        }]
        # 指先をレモン中心よりどれだけ奥まで入れるか。approach はその先 3 cm まで
        # 直線で入れられることを確かめる (insert_margin >= grasp_depth + 0.03)
        approach_params = [{
            "use_sim_time": sim,
            "insert_margin": float(arg("insert_margin")),
        }]
        grasp_params = [{
            "use_sim_time": sim,
            "grasp_depth": float(arg("grasp_depth")),
        }]

        # lemon_grasp_node は Servo を同じプロセスで動かすのでロボットモデルが要る
        moveit_config = MoveItConfigsBuilder(
            "piper", package_name="piper_with_gripper_moveit"
        ).to_moveit_configs()
        grasp_config = os.path.join(
            get_package_share_directory("lemon_grasp"), "config"
        )
        servo_yamls = [os.path.join(grasp_config, "servo.yaml")]

        if not sim:
            target_params.append(REAL_TARGET_PARAMS)
            params.append(REAL_MOVEIT_PARAMS)
            approach_params.append(REAL_MOVEIT_PARAMS)
            grasp_params.append(REAL_GRASP_PARAMS)
            servo_yamls.append(os.path.join(grasp_config, "servo_real.yaml"))

        return [
            Node(
                package="lemon_reach",
                executable="lemon_target_node",
                parameters=target_params,
                output="screen",
            ),
            Node(
                package="lemon_reach",
                executable="lemon_approach_node",
                parameters=approach_params,
                output="screen",
            ),
            Node(
                package="lemon_grasp",
                executable="lemon_grasp_node",
                name="lemon_grasp",
                parameters=[
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    moveit_config.joint_limits,
                    *servo_yamls,
                    *grasp_params,
                ],
                # PoseTracking が購読する target_pose をノードの名前空間に入れる
                remappings=[("target_pose", "/lemon_grasp/target_pose")],
                output="screen",
            ),
            Node(
                package="lemon_reach",
                executable="arm_home_node",
                parameters=params,
                output="screen",
            ),
            Node(
                package="lemon_reach",
                executable="lemon_harvest_manager_node",
                parameters=params,
                output="screen",
                condition=IfCondition(LaunchConfiguration("auto_run")),
            ),
        ]

    return LaunchDescription([
        DeclareLaunchArgument(
            "sim",
            default_value="false",
            description="true: Isaac Sim, false: real robot",
        ),
        DeclareLaunchArgument(
            "auto_run",
            default_value="true",
            description="Whether to run the harvest sequence "
                        "(false: call each node's services manually)",
        ),
        DeclareLaunchArgument(
            "score_threshold",
            default_value="0.5",
            description="Minimum YOLO score used by lemon_target",
        ),
        DeclareLaunchArgument(
            "grasp_depth",
            default_value="0.03",
            description="How far the fingertips go beyond the lemon center [m]",
        ),
        DeclareLaunchArgument(
            "insert_margin",
            default_value="0.06",
            description="Straight insertion checked beyond the lemon center [m] "
                        "(keep >= grasp_depth + 0.03)",
        ),
        OpaqueFunction(function=make_nodes),
    ])
