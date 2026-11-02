# 1つのLemonの検出と、そのマスクに当てはめた楕円
# 3Dの値は LemonEllipseArray の header.frame_id (カメラの光学座標系) で表す

string class_name
float64 score
# yolo の tracking id
string id

# 3D中心 (detect_3d_node の bbox3d 中心。見えている表面の深度)
geometry_msgs/Point center

# 短辺・長辺の3Dの向き (単位ベクトル、視線に垂直な面の中。向きは 180 度で同じ)
geometry_msgs/Vector3 short_axis
geometry_msgs/Vector3 long_axis
# 短辺の向きをグリッパの向きに使えるか
# (長辺/短辺 >= min_aspect かつ 塗りつぶし率 >= min_fill)
bool short_axis_valid

# 画像上の楕円 [px]
float64 center_u
float64 center_v
float64 major_radius
float64 minor_radius
# 短辺が画像の横軸となす角 [rad] (-pi/2, pi/2]、画像上で時計回りが正
float64 minor_angle

# 長辺 / 短辺
float64 aspect
# マスクの面積 / 楕円の面積 (葉などで欠けたマスクほど小さい)
float64 fill
