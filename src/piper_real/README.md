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
# cam_* は 2026-11-11 に実測して、収穫できた値（下の「注意」）
ros2 launch piper_real piper_real_bringup.launch.py use_rviz:=false \
    cam_x:=-0.06 cam_y:=-0.04 cam_z:=0.04 cam_roll:=0.0 cam_pitch:=-1.2217 cam_yaw:=0.0

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

- `cam_*` (link6 → camera_link) は **実測値を必ず入れる**。デフォルトは仮の 0
  (まっすぐ前向き)。RViz で TF と点群が実物と一致しているか確認する。
  - link6 の軸: +z = 前 (差し込む向き)、+x = 下、+y = 後ろから見て左
  - 今の取り付け (2026-11-11 実測): 手首の上 6cm・右 4cm・前 4cm、前を見たまま 20° 下向き
    → `cam_x:=-0.06 cam_y:=-0.04 cam_z:=0.04 cam_pitch:=-1.2217` (−90° + 20°)
  - 既定値のままだとレモンの約 10cm 上を掴みにいった。高さだけずれるのは向き (pitch) が原因
- 実機だけ、`config/arm_controller_real.yaml` で JTC の追従の許容を足している
  (動いている間に指令と実機が 0.25 rad 以上ずれたら中断し、終わりで 0.03 rad 以上離れていたら失敗)。
  piper_joint_bridge は今の位置から 0.3 rad より遠い指令を捨てるので、これが無いと
  途中で止まったのに「着いた」扱いになる。Isaac Sim と共用の `ros2_controllers.yaml` は変えていない
- 起動から5秒後にコントローラが有効化され、その時点の実機の姿勢が保持目標になる。
  piper のフィードバックが来ていないとブリッジが指令を捨てる (ログに出る)。
