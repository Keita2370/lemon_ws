#!/usr/bin/env python3
"""Joystick teleop for both the Piper arm and the Go2 chassis, replacing
joy_to_piper_joint_states.cpp (renamed/rewritten in Python -- that node's own
name was misleading: it published *commands*, not states, on a topic it
called "joint_states"/"joint_ctrl_single" before being hand-patched locally
to "joint_command").

Two modes, selected by holding the mode-switch button (Xbox LB by default):
  - Default: drives the Piper arm/gripper -- left stick X = joint1, left
    stick Y = joint3, right stick Y = joint2, D-pad Y = joint5, D-pad X =
    joint4 (or joint6 with the modifier button held), a toggle button for
    the gripper, and a home button that snaps the arm straight back to
    piper_with_gripper_moveit's SRDF "zero" group_state (all joints 0 --
    the only documented reference/folded pose; no separate "home" pose is
    defined anywhere in piper_ros) plus the gripper open -- publishing on
    joint_command in piper_ctrl_single_node.py's actual real-hardware shape:
    a 7-element JointState, name=['joint1'..'joint6','gripper'], gripper as
    one combined value (matches go2_in_isaacsim's HardwareCompatibleBridge,
    which splits it back into joint7/joint8 on the Isaac Sim side).
  - While the mode-switch button is held: the same two sticks drive Go2's
    cmd_vel (geometry_msgs/Twist) instead -- left stick for linear x/y,
    right stick X for turning, scaled to go2_in_isaacsim's own trained
    command ranges (see go2_example.py: lin_vel_x=[-1,1], lin_vel_y=[-0.4,0.4],
    ang_vel_z=[-1,1]). The arm is left exactly where it was (no joint_command
    published) while in this mode; cmd_vel is actively zeroed while *not* in
    this mode so Go2 doesn't keep coasting on a stale command (go2_in_isaacsim
    has no cmd_vel staleness timeout of its own).

All axis/button indices are ROS2 parameters (defaults match a standard
Xbox controller via the Linux joy driver: axes 0/1=left stick X/Y,
3/4=right stick X/Y, 6/7=D-pad X/Y; buttons 0/1/2/3=A/B/X/Y, 4/5=LB/RB,
7=Start, 8=Guide/Xbox "home" button) -- override them if your pad reports
differently (`ros2 topic echo /joy` while pressing each button/stick will
show which index moves). Note some joy drivers/older kernels don't report
the Guide button at all -- if button_home never seems to fire, check
`ros2 topic echo /joy` for whether pressing it shows up as anything.
"""
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState

_ARM_JOINT_NAMES = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")
_GRIPPER_OPEN = 0.035  # matches piper_with_gripper_moveit's SRDF "open" group_state
_GRIPPER_CLOSED = 0.0
# piper_with_gripper_moveit's SRDF "zero" group_state -- the only documented
# reference/folded pose for the arm anywhere in piper_ros.
_HOME_JOINT_ANGLES = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


class JoyTeleopNode(Node):
    def __init__(self) -> None:
        super().__init__("joy_teleop_node")

        self.declare_parameter("piper_joint_command_topic", "joint_command")
        self.declare_parameter("go2_cmd_vel_topic", "cmd_vel")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("deadzone", 0.2)
        self.declare_parameter("joint_speed", 0.04)  # rad per publish tick, while held past deadzone

        # Axis/button indices -- default to a standard Xbox pad (see module
        # docstring). Same physical sticks are reused for both modes.
        self.declare_parameter("axis_joint1", 0)          # left stick X
        self.declare_parameter("axis_joint2", 4)          # right stick Y
        self.declare_parameter("axis_joint3", 1)          # left stick Y
        self.declare_parameter("axis_dpad_x", 6)
        self.declare_parameter("axis_dpad_y", 7)          # D-pad Y -> joint5
        self.declare_parameter("button_joint6_modifier", 2)   # X: dpad X drives joint6 instead of joint4
        self.declare_parameter("button_gripper_toggle", 0)     # A
        self.declare_parameter("button_grab_pose", 1)          # B
        self.declare_parameter("button_zero_reset", 7)         # Start (arm only, held)
        self.declare_parameter("button_home", 8)                # Guide/Xbox button (arm + gripper, edge-triggered)
        self.declare_parameter("button_go2_mode", 4)           # LB, hold for Go2 mode

        self.declare_parameter("axis_go2_linear_x", 1)     # left stick Y
        self.declare_parameter("axis_go2_linear_y", 0)     # left stick X
        self.declare_parameter("axis_go2_angular_z", 3)    # right stick X
        # Go2FlatTerrainPolicy's own trained command ranges (go2_example.py).
        self.declare_parameter("go2_linear_x_scale", 1.0)
        self.declare_parameter("go2_linear_y_scale", 0.4)
        self.declare_parameter("go2_angular_z_scale", 1.0)

        self._joint_angle = list(_HOME_JOINT_ANGLES)
        self._gripper_value = _GRIPPER_OPEN
        self._is_gripping = False
        self._prev_gripper_button = False
        self._prev_home_button = False
        self._joy = None

        self._joint_pub = self.create_publisher(
            JointState, self.get_parameter("piper_joint_command_topic").value, 1
        )
        self._cmd_vel_pub = self.create_publisher(
            Twist, self.get_parameter("go2_cmd_vel_topic").value, 1
        )
        self.create_subscription(Joy, "joy", self._on_joy, 100)
        rate = self.get_parameter("publish_rate_hz").value
        self._timer = self.create_timer(1.0 / rate, self._on_timer)

    def _on_joy(self, msg: Joy) -> None:
        self._joy = msg

    def _param(self, name: str):
        return self.get_parameter(name).value

    def _axis(self, index: int) -> float:
        if self._joy is None or index >= len(self._joy.axes):
            return 0.0
        return self._joy.axes[index]

    def _button(self, index: int) -> bool:
        if self._joy is None or index >= len(self._joy.buttons):
            return False
        return bool(self._joy.buttons[index])

    def _on_timer(self) -> None:
        if self._joy is None:
            return

        if self._button(self._param("button_go2_mode")):
            self._drive_go2()
        else:
            self._cmd_vel_pub.publish(Twist())  # actively zero -- no staleness timeout on the Go2 side
            self._drive_piper()

    def _drive_go2(self) -> None:
        twist = Twist()
        twist.linear.x = self._axis(self._param("axis_go2_linear_x")) * self._param("go2_linear_x_scale")
        # Negated -- same physical left stick X as axis_joint1 below, which
        # was confirmed backwards live (right/left swapped) on this pad.
        twist.linear.y = -self._axis(self._param("axis_go2_linear_y")) * self._param("go2_linear_y_scale")
        twist.angular.z = self._axis(self._param("axis_go2_angular_z")) * self._param("go2_angular_z_scale")
        self._cmd_vel_pub.publish(twist)

    def _drive_piper(self) -> None:
        deadzone = self._param("deadzone")
        speed = self._param("joint_speed")
        modifier = self._button(self._param("button_joint6_modifier"))

        # Negated -- confirmed backwards live (right/left swapped) on this pad.
        axis1 = -self._axis(self._param("axis_joint1"))
        if axis1 > deadzone:
            self._joint_angle[0] = max(-2.618, self._joint_angle[0] - speed)
        elif axis1 < -deadzone:
            self._joint_angle[0] = min(2.168, self._joint_angle[0] + speed)

        axis2 = self._axis(self._param("axis_joint2"))
        if axis2 > deadzone:
            self._joint_angle[1] = min(3.14, self._joint_angle[1] + speed)
        elif axis2 < -deadzone:
            self._joint_angle[1] = max(0.0, self._joint_angle[1] - speed)

        # Left stick Y -- direct control of joint3.
        axis3 = self._axis(self._param("axis_joint3"))
        if axis3 > deadzone:
            self._joint_angle[2] = max(-2.967, self._joint_angle[2] - speed)
        elif axis3 < -deadzone:
            self._joint_angle[2] = min(0.0, self._joint_angle[2] + speed)

        # D-pad Y -- direct control of joint5.
        dpad_y = self._axis(self._param("axis_dpad_y"))
        if dpad_y == 1:
            self._joint_angle[4] = max(-1.220, self._joint_angle[4] - speed)
        elif dpad_y == -1:
            self._joint_angle[4] = min(1.220, self._joint_angle[4] + speed)

        # Negated -- same left/right reversal as the D-pad's own hat X axis
        # on this pad.
        dpad_x = -self._axis(self._param("axis_dpad_x"))
        if dpad_x == 1:
            if modifier:
                self._joint_angle[5] = max(-2.094, self._joint_angle[5] - speed)
            else:
                self._joint_angle[3] = max(-1.745, self._joint_angle[3] - speed)
        elif dpad_x == -1:
            if modifier:
                self._joint_angle[5] = min(2.094, self._joint_angle[5] + speed)
            else:
                self._joint_angle[3] = min(1.745, self._joint_angle[3] + speed)

        gripper_button = self._button(self._param("button_gripper_toggle"))
        if gripper_button and not self._prev_gripper_button:
            self._is_gripping = not self._is_gripping
            self._gripper_value = _GRIPPER_CLOSED if self._is_gripping else _GRIPPER_OPEN
        self._prev_gripper_button = gripper_button

        if self._button(self._param("button_zero_reset")):
            self._joint_angle = [0.0] * 6

        home_button = self._button(self._param("button_home"))
        if home_button and not self._prev_home_button:
            self._joint_angle = list(_HOME_JOINT_ANGLES)
            self._is_gripping = False
            self._gripper_value = _GRIPPER_OPEN
        self._prev_home_button = home_button

        if self._button(self._param("button_grab_pose")):
            self._joint_angle[1] = 0.950
            self._joint_angle[2] = -1.315
            self._joint_angle[4] = 1.220

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(_ARM_JOINT_NAMES) + ["gripper"]
        msg.position = list(self._joint_angle) + [self._gripper_value]
        self._joint_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JoyTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
