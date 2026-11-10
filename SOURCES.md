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

## 2026-11-02 に加えた変更（UI から操作するための口）

lemon 収穫側の担当メンバーの了承を得て、UI（`Keita2370/piper_arm_ui` の `backend:=lemon`）から
操作するために足した。**呼ばない限り・指定しない限り、従来の動きは変わらない**。
元のリポジトリ（hida-yat/lemon_reach）には push していない。

| パッケージ | 変更 |
|---|---|
| `lemon_grasp` | **`~/stop` サービス**を追加。差し込み中は Servo を止め、閉じる・戻りの最中は JTC のゴールを取り消す。止めたら閉じる・戻る段は飛ばし、`stopped (while ...)` を返す（アームはその場に残る）|
| `lemon_msgs` | **`srv/SelectTarget.srv`** を追加（`geometry_msgs/PointStamped` → `success`, `message`）|
| `lemon_reach` / `lemon_target_node.py` | **`~/select`** と **`selection_mode`**（`auto` 既定 / `manual`）。manual では自動で選ばず、指定位置に一番近い安定したレモンを固定。固定中は断る。安定した候補が無いときは理由を返す |
| `lemon_reach` / `lemon_approach_node.py` | `no target` / `failed to compute pregrasp` / `cannot insert ...` も `~/status` に出す（`busy` は出さない）|
| `lemon_reach` / `lemon_harvest.launch.py` | 引数 `selection_mode` を追加 |
| `lemon_reach` / `docs/lemon_harvest_system.md` | 上の口の説明を追記 |

実機で確認済み（`~/stop` は差し込み中に呼んで 0.04 s で返り、目視でもすぐ止まった。
`~/select` は位置の指定・遠い位置・固定中の拒否・除外リストの無視を確認）。

## 2026-11-10 にミニPCで加えられた変更（home / 認識姿勢 / 回収姿勢）

ミニPCの `~/lemon_ws/src/lemon_reach` に直接加えられた変更を、そのまま記録したもの
（ファイルの日付は 2026-11-10 18:49。`config/` のインストールは 2026-11-05）。

| ファイル | 変更 |
|---|---|
| `config/harvest_sequence.yaml`（新規） | home・observe（右／左）・collect の関節角 |
| `arm_home_node.py` | home = 犬（Go2）が移動するときの畳んだ姿勢。observe（レモン認識姿勢、右／左）と collect（回収姿勢）を追加。`~/go_observe(_right/_left)`、`~/set_observe(_right/_left)`、`~/go_collect`、`~/set_collect`。`~/harvest_enabled`（Bool、latched）と `~/observe_side`（String、latched）を配信 |
| `lemon_target_node.py` | `harvest_enabled` が false の間は対象を選ばず、`~/select` を断る。false になったら対象を解除し、除外リストを消す |
| `lemon_approach_node.py` | `harvest_enabled` が false の間は `~/approach` を断る |
| `lemon_harvest_manager_node.py` | 起動時に observe へ。把持に成功したら collect で離して observe へ戻る。失敗したら observe へ戻ってから開く |
| `launch/lemon_harvest.launch.py`、`setup.py` | `config/harvest_sequence.yaml` を読む・入れる |
| `docs/lemon_harvest_system.*` | 上の説明 |
