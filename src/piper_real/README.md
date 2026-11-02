# piper_real

実機の Piper (CAN) を MoveIt から動かすための bringup。
`piper_isaacsim` と同じ構成 (topic_based_ros2_control + arm/gripper
controllers + move_group) で、Isaac Sim の代わりに piper_ros の
`piper_single_ctrl` につなぐ。

```
move_group ─ arm_controller ─ topic_based_ros2_control
                                   │ /piper_joint_bridge/joint_command
                                   ▼
                          piper_joint_bridge ── /joint_ctrl_single ──► piper_single_ctrl ─ CAN ─ Piper
                                   ▲
                                   │ /piper_joint_bridge/joint_states
                          piper_joint_bridge ◄── /joint_states_single ── piper_single_ctrl
```

`piper_joint_bridge` は gripper (開き幅) ⇔ joint7/joint8 の変換と、
現在角から `max_jump` (0.3 rad) 以上離れた指令を捨てる安全ガードを行う。

## Setup (初回のみ)

```bash
sudo apt install ros-humble-topic-based-ros2-control ros-humble-realsense2-camera
pip3 install ultralytics          # yolo_ros 用 (このPCはCPU推論)
cd ~/lemon_ws && colcon build --symlink-install
```

## Run

```bash
# 1. CAN 有効化
cd ~/piper_ros && bash can_activate.sh can0 1000000

# 2. アーム + MoveIt + 手首カメラ
ros2 launch piper_real piper_real_bringup.launch.py \
    cam_x:=... cam_y:=... cam_z:=... cam_roll:=... cam_pitch:=... cam_yaw:=...

# (ssh 越しなら use_rviz:=false にして、RViz は手元のPCで:
#  ros2 launch piper_real piper_real_rviz.launch.py)

# 3. レモン収穫
ros2 launch lemon_reach lemon_harvest.launch.py          # sim:=true で Isaac Sim 用
```

## 注意

- `control_rate` (デフォルト 100Hz) は piper_single_ctrl の publish 周期と
  ros2_control の update_rate。元の 200Hz は N95 では CPU を食いすぎる。
  piper_single_ctrl 側は `~/piper_ros` に足した `publish_rate` パラメータで渡す。
- `use_octomap` はデフォルト false (点群→OctoMap が重い)。障害物回避が要るときだけ true。
- ssh の X11 転送で RViz を出すと起動に約50秒かかり操作もほぼ不能。
  手元のPCで `piper_real_rviz.launch.py` を使う (同じ LAN・同じ ROS_DOMAIN_ID)。

- `cam_*` (link6 → camera_link) は **実測値を必ず入れる**。デフォルトは仮の 0。
  RViz で TF と点群が実物と一致しているか確認する。
- 起動から5秒後にコントローラが有効化され、その時点の実機の姿勢が保持目標になる。
  piper のフィードバックが来ていないとブリッジが指令を捨てる (ログに出る)。
