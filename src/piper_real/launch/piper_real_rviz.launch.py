"""RViz (MoveIt MotionPlanning パネル付き) だけを起動する。

ロボットPCで piper_real_bringup.launch.py use_rviz:=false を動かし、
こちらは手元のPCで起動する (ssh の X11 転送で RViz を出すと非常に重いため)。
同じ LAN・同じ ROS_DOMAIN_ID なら DDS で move_group / TF / カメラ画像につながる。

手元のPCに必要なもの: ROS 2 Humble + MoveIt (ros-humble-moveit) と、
ビルド済みの piper_description / piper_with_gripper_moveit / piper_real。
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    moveit_config = MoveItConfigsBuilder(
        "piper", package_name="piper_with_gripper_moveit"
    ).to_moveit_configs()

    # bringup の rviz Node と同じ。kinematics が無いと
    # インタラクティブマーカーをドラッグしても IK が解けない
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=[
            "-d",
            str(moveit_config.package_path / "config" / "moveit.rviz"),
        ],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
    )

    return LaunchDescription([rviz])
