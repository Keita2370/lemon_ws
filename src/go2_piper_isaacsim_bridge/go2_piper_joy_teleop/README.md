# go2_piper_joy_teleop

Joystick teleop for both the Piper arm (via `joint_command`, real-hardware
7-element `sensor_msgs/JointState` shape) and the Go2 chassis (via
`cmd_vel`), switching between them with a hold button. Replaces
`joy_to_piper_joint_states` (renamed/rewritten in Python -- that node's own
name was misleading, since it published *commands*, and it only drove
Piper, not Go2).

Targets the `go2_in_isaacsim` Isaac Sim extension's Piper
`HardwareCompatibleBridge` (`piper.py`) on the receiving end: `joint_command`
in, `joint_states_single` out, `name=['joint1'..'joint6','gripper']`, gripper
as one combined value.

## Run

```bash
source install/setup.bash
ros2 launch go2_piper_joy_teleop joy_teleop.launch.py
```

Starts `joy_node` (package `joy`) and `joy_teleop_node` together.

## Controls (default: standard Xbox pad via the Linux joy driver)

Default mode drives the Piper arm/gripper:

| Input | Action |
|---|---|
| Left stick X | joint1 |
| Left stick Y | joint3 |
| Right stick Y | joint2 |
| D-pad Y | joint5 |
| D-pad X | joint4 (joint6 if X held) |
| A | toggle gripper open/close |
| B | "grab" pose preset |
| Start | force arm to all-zero (held) |
| Guide/Xbox button | snap arm to home (all-zero) + gripper open (edge-triggered) |

Holding **LB** switches both sticks to driving Go2's `cmd_vel` instead
(left stick = linear x/y, right stick X = turn); the arm is left exactly
where it was while in this mode, and `cmd_vel` is actively zeroed while
*not* in this mode (go2_in_isaacsim has no `cmd_vel` staleness timeout of
its own).

All the above indices, plus topic names, deadzone, speed, and Go2 command
scales, are ROS2 parameters on `joy_teleop_node` -- see the top of
`joy_teleop_node.py` for the full list and defaults. If your pad reports
differently, `ros2 topic echo /joy` while pressing each button/stick shows
which index moves; override with e.g.
`--ros-args -p button_home:=<index>`.

## Known limitations

- No documented "home"/folded pose exists anywhere in piper_ros -- the home
  button snaps to the SRDF's all-zero `"zero"` group_state, the only
  documented reference pose. If that's not actually a folded configuration
  on your arm, edit `_HOME_JOINT_ANGLES` in `joy_teleop_node.py`.
- The physical Guide/Xbox "home" button isn't reported by every joy
  driver/kernel version -- check `ros2 topic echo /joy` if it never seems to
  fire.
- Not tested against real Piper hardware, only against
  go2_in_isaacsim's `HardwareCompatibleBridge`.
