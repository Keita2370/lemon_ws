import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Isaac Sim (go2_in_isaacsim) と実機 (piper_real が起動する realsense2_camera) の違い
CAMERA = {
    "sim": {
        "yolo": {
            "input_image_topic": "/realsense/color/image_raw",
            "input_depth_topic": "/realsense/depth/image_rect_raw",
            "input_depth_info_topic": "/realsense/color/camera_info",
            "target_frame": "d435_color_optical_frame",
            # Isaac Simの深度は 32FC1 [m]
            "depth_image_units_divisor": "1",
        },
        "camera_info_topic": "/realsense/color/camera_info",
        "device": "cuda:0",
    },
    "real": {
        "yolo": {
            "input_image_topic": "/camera/camera/color/image_raw",
            # マスク内の深度を取るので color に揃えた深度を使う (align_depth.enable:=true)
            "input_depth_topic": "/camera/camera/aligned_depth_to_color/image_raw",
            "input_depth_info_topic": "/camera/camera/color/camera_info",
            "target_frame": "camera_color_optical_frame",
            # RealSenseの深度は 16UC1 [mm]
            "depth_image_units_divisor": "1000",
            # このPCはGPUなし (Intel N95) なので推論を軽くする
            "imgsz_height": "256",
            "imgsz_width": "320",
            # lemon_target_node はトラッキングIDを使わない
            "use_tracking": "False",
        },
        "camera_info_topic": "/camera/camera/color/camera_info",
        "device": "cpu",
    },
}


def generate_launch_description():
    """
    手首の D435 でLemonを検出する yolo_ros を起動する (lemon_harvest.launch.py とは別に起動)。

    セグメンテーションモデルを使うと、detect_3d_node は深度をマスク内の画素だけから
    取るので、葉や背景の深度が混ざりにくい。3D位置はカメラ座標のまま出させ、
    base_link への変換は lemon_target_node が撮影時刻のTFで行う
    (detect_3d_node は最新のTFで変換するため)。

    lemon_ellipse_node が検出ごとにマスクへ楕円を当てはめて短辺・長辺を求め、
    /lemon_ellipse/lemons に配信する (lemon_target の入力なので常に起動する)。
    短辺・長辺は YOLO の debug 画像に重ねた /lemon_ellipse/dbg_image と、
    RViz 用の /lemon_ellipse/markers で表示する。

    sim:=true で Isaac Sim、false (既定) で実機のカメラを使う。
    """

    def make_nodes(context):

        sim = LaunchConfiguration("sim").perform(context).lower() == "true"
        camera = CAMERA["sim" if sim else "real"]

        device = LaunchConfiguration("device").perform(context) or camera["device"]

        launch_arguments = dict(camera["yolo"])
        launch_arguments.update({
            "model": LaunchConfiguration("model").perform(context),
            "device": device,
            "threshold": LaunchConfiguration("threshold").perform(context),
            "use_3d": "True",
        })

        yolo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("yolo_bringup"),
                    "launch",
                    "yolo.launch.py",
                )
            ),
            launch_arguments=launch_arguments.items(),
        )

        ellipse = Node(
            package="lemon_reach",
            executable="lemon_ellipse_node",
            parameters=[{
                "use_sim_time": sim,
                "camera_info_topic": camera["camera_info_topic"],
                "publish_debug_image": LaunchConfiguration("publish_debug_image"),
            }],
            output="screen",
        )

        return [yolo, ellipse]

    return LaunchDescription([
        DeclareLaunchArgument(
            "sim",
            default_value="false",
            description="true: Isaac Sim, false: real robot",
        ),
        DeclareLaunchArgument(
            "model",
            default_value=os.path.expanduser("~/lemon_ws/best.pt"),
            description="YOLO segmentation model (class 'lemon')",
        ),
        DeclareLaunchArgument(
            "device",
            default_value="",
            description="Inference device (empty: cuda:0 for sim, cpu for real)",
        ),
        DeclareLaunchArgument("threshold", default_value="0.5"),
        DeclareLaunchArgument(
            "publish_debug_image",
            default_value="true",
            description="Publish the ellipse overlay on /lemon_ellipse/dbg_image",
        ),
        OpaqueFunction(function=make_nodes),
    ])
