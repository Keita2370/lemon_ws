#!/usr/bin/env python3

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from sensor_msgs.msg import JointState


ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]


class PiperJointBridge(Node):
    """
    topic_based_ros2_control と piper_ros の piper_single_ctrl の間の変換。

    piper_single_ctrl のグリッパは "gripper" (開き幅 [m], 0~0.07) の1軸だが、
    URDF/MoveIt は joint7 (0~0.035) / joint8 (-0.035~0) の左右2軸なので変換する。

        /joint_states_single (piper) → ~/joint_states (topic_based_ros2_control)
            gripper → joint7 = w/2, joint8 = -w/2
        ~/joint_command (topic_based_ros2_control) → /joint_ctrl_single (piper)
            joint7 → gripper = 2 * joint7

    joint_offsets [rad] (joint1~6) で実機とモデルの関節角のずれを補正する
    (取り付けのずれで、実機の 0 がモデルの 0 と合わない関節がある場合)。

        モデルの関節角 = 実機の関節角 + offset

    安全のため、現在の実機の関節角から max_jump [rad] 以上離れた指令は転送しない
    (フィードバックが来る前に有効化されたコントローラが 0 を指令し、
    アームが急に0姿勢へ動くのを防ぐ)。
    """

    def __init__(self):
        super().__init__("piper_joint_bridge")

        self.declare_parameter("max_jump", 0.3)
        self.declare_parameter("gripper_max", 0.07)
        self.declare_parameter("joint_offsets", [0.0] * len(ARM_JOINTS))

        self.max_jump = self.get_parameter("max_jump").value
        self.gripper_max = self.get_parameter("gripper_max").value

        offsets = list(self.get_parameter("joint_offsets").value)
        if len(offsets) != len(ARM_JOINTS):
            raise ValueError(f"joint_offsets must have {len(ARM_JOINTS)} values")
        self.offsets = dict(zip(ARM_JOINTS, offsets))

        if any(offsets):
            self.get_logger().info(
                "Joint offsets [deg]: " + ", ".join(
                    f"{n}={math.degrees(v):.1f}"
                    for n, v in self.offsets.items() if v
                )
            )

        self.current = None     # モデル側の関節角 {name: position}

        self.state_pub = self.create_publisher(JointState, "~/joint_states", 10)
        self.ctrl_pub = self.create_publisher(JointState, "/joint_ctrl_single", 10)

        self.create_subscription(
            JointState, "/joint_states_single", self.feedback_callback, 10
        )
        self.create_subscription(
            JointState, "~/joint_command", self.command_callback, 10
        )

    def feedback_callback(self, msg):

        positions = dict(zip(msg.name, msg.position))

        if any(n not in positions for n in ARM_JOINTS):
            return

        for n in ARM_JOINTS:
            positions[n] += self.offsets[n]

        self.current = positions

        width = positions.get("gripper", 0.0)

        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.name = ARM_JOINTS + ["joint7", "joint8"]
        out.position = (
            [float(positions[n]) for n in ARM_JOINTS]
            + [width / 2.0, -width / 2.0]
        )
        # piper_single_ctrl の velocity は6要素しかないので名前で揃え直す
        velocities = dict(zip(msg.name, msg.velocity))
        out.velocity = (
            [float(velocities.get(n, 0.0)) for n in ARM_JOINTS] + [0.0, 0.0]
        )

        self.state_pub.publish(out)

    def command_callback(self, msg):

        if self.current is None:
            self.get_logger().warn(
                "No feedback from piper yet, command dropped",
                throttle_duration_sec=1.0
            )
            return

        positions = dict(zip(msg.name, msg.position))

        if any(n not in positions for n in ARM_JOINTS):
            return

        for n in ARM_JOINTS:
            if not math.isfinite(positions[n]):
                return

            if abs(positions[n] - self.current[n]) > self.max_jump:
                self.get_logger().error(
                    f"Command {n}={positions[n]:.3f} is too far from "
                    f"current {self.current[n]:.3f}, dropped",
                    throttle_duration_sec=1.0
                )
                return

        gripper = 2.0 * positions.get(
            "joint7", self.current.get("gripper", 0.0) / 2.0
        )
        gripper = min(max(gripper, 0.0), self.gripper_max)

        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.name = ARM_JOINTS + ["gripper"]
        out.position = (
            [float(positions[n] - self.offsets[n]) for n in ARM_JOINTS]
            + [gripper]
        )

        self.ctrl_pub.publish(out)


def main():

    rclpy.init()

    node = PiperJointBridge()

    try:
        rclpy.spin(node)

    except (KeyboardInterrupt, ExternalShutdownException):
        pass

    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
