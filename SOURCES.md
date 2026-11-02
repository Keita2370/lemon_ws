# lemon_ws（ミニPC の ~/lemon_ws のスナップショット）

2026-11-02 時点の、実機ミニPC上の `~/lemon_ws` をそのまま写したもの。
`build/` `install/` `log/` と `src/yolo_ros/yolo_ros/.venv`（Python 仮想環境、約12GB）は含めていない。

## 各パッケージの出どころ

| パッケージ | 元のリポジトリ | コミット | 手元の未コミットの変更（このスナップショットに含む） |
|---|---|---|---|
| `go2_piper_isaacsim_bridge` | https://github.com/hida-yat/go2_piper_isaacsim_bridge.git | `13ae348` (2026-09-11) | ?? go2_piper_joy_teleop/COLCON_IGNORE |
| `lemon_reach` | https://github.com/hida-yat/lemon_reach.git | `c784356` (2026-10-05) | M launch/lemon_harvest.launch.py, M launch/lemon_yolo.launch.py, ?? docs/ |
| `yolo_ros` | https://github.com/mgonzs13/yolo_ros.git | `eb11e81` (2026-09-05) | M yolo_bringup/launch/yolov8.launch.py |
| `lemon_grasp` | （git 管理外） | — | — |
| `lemon_msgs` | （git 管理外） | — | — |
| `piper_real` | （git 管理外） | — | — |

`best.pt` / `lemon.pt` は YOLO のモデル。
