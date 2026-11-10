#!/usr/bin/env python3

import math

import rclpy
from rclpy.callback_groups import (
    MutuallyExclusiveCallbackGroup,
    ReentrantCallbackGroup,
)
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node

from rclpy.qos import QoSProfile, QoSDurabilityPolicy

from moveit_msgs.msg import Constraints, JointConstraint
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from lemon_reach.moveit_helper import MoveItHelper


class ArmHome(Node):
    """
    起動時のアームの関節角をhome姿勢として記録し、そこへ戻す。

    ~/go_home  (Trigger): home姿勢へ計画・実行する
    ~/set_home (Trigger): 現在の姿勢をhome姿勢として記録し直す

    [追加] config/harvest_sequence.yaml で関節角 [rad] を与える。
    home    : 犬 (Go2) が移動するときの畳んだ姿勢。収穫の一連の動作には使わない
              home_joint_positions を与えなければ、従来どおり起動時の姿勢を記録する
    observe : レモン認識姿勢 (手先カメラが作業範囲を向く)。収穫の合間に戻る姿勢。
              右と左の2つ。observe_right_joint_positions / observe_left_joint_positions
              (未設定の側へは行けない)。今の向き (~/observe_side、起動時は
              observe_side パラメータ、既定 right) を覚えておき、~/go_observe はその向きへ行く
    ~/go_observe        (Trigger): 今の向きの認識姿勢へ計画・実行する (manager もこれを呼ぶ)
    ~/go_observe_right  (Trigger): 向きを右にして、右の認識姿勢へ計画・実行する
    ~/go_observe_left   (Trigger): 向きを左にして、左の認識姿勢へ計画・実行する
    ~/set_observe       (Trigger): 現在の姿勢を今の向きの認識姿勢として記録し直す
    ~/set_observe_right / ~/set_observe_left (Trigger): 現在の姿勢を右 / 左の認識姿勢として記録し直す
        (どれも YAML に貼れる形でログに出す)
    ~/observe_side (std_msgs/String, latched): 今の向き ("right" / "left")
    collect : レモン回収姿勢 (auto_run でグリッパを開いてレモンを離す姿勢)
              collect_joint_positions (未設定なら ~/go_collect は失敗する)
    ~/go_collect  (Trigger): 回収姿勢へ計画・実行する (harvest_enabled は変えない)
    ~/set_collect (Trigger): 現在の姿勢を回収姿勢として記録し直す (YAMLに貼れる形でログに出す)

    ~/harvest_enabled (std_msgs/Bool, latched): 収穫してよいか。
        起動時 false、~/go_observe (_right / _left も) が成功したら true、~/go_home が呼ばれたら
        移動の前に false (手先カメラなので home ではレモンが映らない)。
        lemon_target / lemon_approach はこれが false の間、収穫操作を断る
    """

    def __init__(self):
        super().__init__("arm_home")

        self.cb_group = ReentrantCallbackGroup()
        self.moveit = MoveItHelper(self, self.cb_group)

        self.declare_parameter(
            "joint_names",
            ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        )
        self.declare_parameter("joint_tolerance", 0.01)
        # [追加] 設定ファイルで与える姿勢 [rad]。既定値のNaNは「未設定」の意味
        # (空リストだと型がDOUBLE_ARRAYに決まらず、設定ファイルの値で上書きできないため)
        self.declare_parameter("home_joint_positions", [math.nan] * 6)
        self.declare_parameter("observe_right_joint_positions", [math.nan] * 6)
        self.declare_parameter("observe_left_joint_positions", [math.nan] * 6)
        # 起動時の認識姿勢の向き (right / left)
        self.declare_parameter("observe_side", "right")
        self.declare_parameter("collect_joint_positions", [math.nan] * 6)

        self.joint_names = self.get_parameter("joint_names").value
        self.joint_tolerance = self.get_parameter("joint_tolerance").value

        self.home = None
        self.busy = False

        # [追加] 設定ファイルの姿勢を読む (homeがあれば起動時の記録は行われない)
        self.home = self.load_pose("home")
        self.observes = {
            side: self.load_pose(f"observe_{side}") for side in ("right", "left")
        }
        self.observe_side = self.get_parameter("observe_side").value.lower()

        if self.observe_side not in self.observes:
            raise ValueError(
                f"observe_side must be right or left: {self.observe_side}"
            )
        self.collect = self.load_pose("collect")

        # [追加] 収穫してよいか (起動時は false)
        latched = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        self.harvest_enabled_pub = self.create_publisher(
            Bool, "~/harvest_enabled", latched
        )
        self.publish_harvest_enabled(False)

        self.observe_side_pub = self.create_publisher(
            String, "~/observe_side", latched
        )
        self.observe_side_pub.publish(String(data=self.observe_side))

        self.create_service(
            Trigger, "~/go_home", self.go_home_callback,
            callback_group=self.cb_group
        )
        self.create_service(
            Trigger, "~/set_home", self.set_home_callback,
            callback_group=self.cb_group
        )
        # [追加]
        self.create_service(
            Trigger, "~/go_observe", self.go_observe_callback,
            callback_group=self.cb_group
        )
        self.create_service(
            Trigger, "~/set_observe", self.set_observe_callback,
            callback_group=self.cb_group
        )
        for side in ("right", "left"):
            self.create_service(
                Trigger, f"~/go_observe_{side}",
                lambda req, res, side=side: self.go_observe_side(side, res),
                callback_group=self.cb_group
            )
            self.create_service(
                Trigger, f"~/set_observe_{side}",
                lambda req, res, side=side: self.set_observe_side(side, res),
                callback_group=self.cb_group
            )
        self.create_service(
            Trigger, "~/go_collect", self.go_collect_callback,
            callback_group=self.cb_group
        )
        self.create_service(
            Trigger, "~/set_collect", self.set_collect_callback,
            callback_group=self.cb_group
        )

        # move_groupが立ち上がるまで記録を再試行する (前回の試行と重ならないように)
        self.record_timer = self.create_timer(
            1.0, self.record_on_startup,
            callback_group=MutuallyExclusiveCallbackGroup()
        )

    def record_on_startup(self):

        if self.home is not None:
            self.record_timer.cancel()
            return

        # [追加] 移動中の姿勢を home にしない
        if self.busy:
            return

        if self.record_home():
            self.record_timer.cancel()

    def record_home(self):

        joints = self.moveit.get_joint_positions()

        if joints is None:
            return False

        missing = [n for n in self.joint_names if n not in joints]

        if missing:
            self.get_logger().error(f"Joints not found: {missing}")
            return False

        self.home = {n: joints[n] for n in self.joint_names}

        self.get_logger().info(
            "Home recorded: "
            + ", ".join(f"{n}={v:.3f}" for n, v in self.home.items())
        )

        return True

    def set_home_callback(self, request, response):

        response.success = self.record_home()
        response.message = "recorded" if response.success else "failed"

        return response

    def go_home_callback(self, request, response):

        # [追加] 動き始める前に収穫を止める (失敗しても false のまま)
        self.publish_harvest_enabled(False)

        return self.move_to(self.home, "home", response)

    def move_to(self, pose, label, response):

        if pose is None:
            response.success = False
            response.message = f"{label} is not set"
            return response

        if self.busy:
            response.success = False
            response.message = "busy"
            return response

        self.busy = True

        # [追加] home が未記録なら動く前に記録する (起動直後に manager が
        # go_observe を呼ぶと、起動時の記録が移動中の姿勢になるため)
        if self.home is None:
            self.record_home()

        try:
            constraints = Constraints()

            for name, position in pose.items():
                jc = JointConstraint()
                jc.joint_name = name
                jc.position = position
                jc.tolerance_above = self.joint_tolerance
                jc.tolerance_below = self.joint_tolerance
                jc.weight = 1.0
                constraints.joint_constraints.append(jc)

            success, message = self.moveit.plan_and_execute(
                constraints, label
            )

        finally:
            self.busy = False

        response.success = success
        response.message = message

        if success:
            self.get_logger().info(f"Reached {label}")
        else:
            self.get_logger().error(f"Go {label} failed: {message}")

        return response

    # --------------------------------------------------
    # [追加] 設定ファイルの姿勢と認識姿勢

    def load_pose(self, name):

        values = list(self.get_parameter(f"{name}_joint_positions").value)

        # 既定値 (すべてNaN) のままなら未設定
        if all(math.isnan(v) for v in values):
            self.get_logger().info(f"{name}_joint_positions is not set")
            return None

        # 値はあるが個数が違う・NaNが混ざっている場合は書き間違いとして警告する
        if (len(values) != len(self.joint_names)
                or any(math.isnan(v) for v in values)):
            self.get_logger().warn(
                f"{name}_joint_positions must have "
                f"{len(self.joint_names)} values: {values}. Ignored"
            )
            return None

        pose = dict(zip(self.joint_names, values))
        self.get_logger().info(
            f"{name} loaded: "
            + ", ".join(f"{n}={v:.3f}" for n, v in pose.items())
        )

        return pose

    def record_pose(self, name, response):
        """現在の姿勢を返し、設定ファイルにそのまま貼れる形でログに出す。取れなければ None"""

        joints = self.moveit.get_joint_positions()

        if joints is None:
            response.success = False
            response.message = "failed"
            return None

        missing = [n for n in self.joint_names if n not in joints]

        if missing:
            self.get_logger().error(f"Joints not found: {missing}")
            response.success = False
            response.message = "failed"
            return None

        pose = {n: joints[n] for n in self.joint_names}

        values = ", ".join(f"{pose[n]:.4f}" for n in self.joint_names)
        self.get_logger().info(f"{name}_joint_positions: [{values}]")

        response.success = True
        response.message = "recorded"

        return pose

    def set_observe_callback(self, request, response):

        return self.set_observe_side(self.observe_side, response)

    def set_observe_side(self, side, response):

        pose = self.record_pose(f"observe_{side}", response)

        if pose is not None:
            self.observes[side] = pose

        return response

    def set_collect_callback(self, request, response):

        pose = self.record_pose("collect", response)

        if pose is not None:
            self.collect = pose

        return response

    def go_collect_callback(self, request, response):

        # 収穫の途中 (把持したレモンを運ぶ) なので harvest_enabled は変えない
        # (false にすると lemon_target が対象と除外リストを消してしまう)
        return self.move_to(self.collect, "collect", response)

    def go_observe_callback(self, request, response):

        return self.go_observe_side(self.observe_side, response)

    def go_observe_side(self, side, response):

        pose = self.observes[side]

        # 姿勢がある側にだけ切り替える (未設定なら move_to が失敗を返す)
        if pose is not None and side != self.observe_side:
            self.observe_side = side
            self.observe_side_pub.publish(String(data=side))
            self.get_logger().info(f"observe_side: {side}")

        moved = pose is not None and not self.busy
        response = self.move_to(pose, f"observe_{side}", response)

        # 認識姿勢に着いたときだけ収穫を許す
        # (動いて失敗したらアームがどこにいるか分からないので false。
        #  未設定・busy でアームが動かなかったときは変えない)
        if response.success or moved:
            self.publish_harvest_enabled(response.success)

        return response

    def publish_harvest_enabled(self, enabled):

        self.harvest_enabled_pub.publish(Bool(data=bool(enabled)))
        self.get_logger().info(f"harvest_enabled: {bool(enabled)}")


def main():

    rclpy.init()

    node = ArmHome()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()

    except (KeyboardInterrupt, ExternalShutdownException):
        pass

    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()