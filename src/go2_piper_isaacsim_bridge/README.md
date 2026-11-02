# go2_piper_isaacsim_bridge

ROS2 packages bridging a Unitree Go2 + Piper arm running in Isaac Sim (the
`go2_in_isaacsim` extension) to the rest of a normal ROS2 stack. Meant to be
dropped into a colcon workspace's `src/` alongside
[piper_ros](https://github.com/agilexrobotics/piper_ros) (both packages
depend on `piper_description` / `piper_with_gripper_moveit` from there) --
not a piper_ros fork, just colocated for building.

## Packages

- **[go2_piper_joy_teleop](go2_piper_joy_teleop/)** -- joystick teleop for
  both the Piper arm/gripper and the Go2 chassis, mode-switched by a hold
  button.
- **[piper_isaacsim](piper_isaacsim/)** -- lets MoveIt
  (`piper_with_gripper_moveit`) drive the Piper arm running in Isaac Sim,
  the same way `piper_gazebo` does for Gazebo, via `topic_based_ros2_control`.

See each package's own README for setup/run/known-limitations detail.

## Build

```bash
colcon build --packages-select go2_piper_joy_teleop piper_isaacsim
```
