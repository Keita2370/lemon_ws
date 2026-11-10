"""Bringup for driving the real Piper arm from MoveIt.

piper_isaacsim/launch/piper_isaacsim_bringup.launch.py と同じ構成
(robot_state_publisher + ros2_control_node(topic_based_ros2_control)
+ controllers + move_group + RViz) で、Isaac Sim の代わりに

    piper_single_ctrl (CAN) <-> piper_joint_bridge <-> topic_based_ros2_control

へつなぐ。手首の RealSense D435 (realsense2_camera) と、link6 → camera_link
の静的TF (取り付け位置は実測値を cam_* 引数で与える) も起動する。

Precondition: CAN が有効になっていること (piper_ros/can_activate.sh)。
"""
import math
import os

import xacro
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    pkg_share = FindPackageShare(package="piper_real").find("piper_real")
    isaacsim_share = FindPackageShare(package="piper_isaacsim").find("piper_isaacsim")

    # ros2_control の xacro / controllers は piper_isaacsim のものをそのまま使う
    # (joint topic を引数で差し替えられるので実機でも同じものでよい)
    xacro_file = os.path.join(isaacsim_share, "config", "piper_isaacsim.urdf.xacro")
    controllers_yaml = os.path.join(isaacsim_share, "config", "ros2_controllers.yaml")
    # 実機だけの arm_controller の設定 (到達の許容)。controllers_yaml の後に重ねる
    arm_controller_real_yaml = os.path.join(pkg_share, "config", "arm_controller_real.yaml")
    sensors_yaml = os.path.join(pkg_share, "config", "sensors_3d.yaml")

    args = [
        DeclareLaunchArgument("can_port", default_value="can0"),
        DeclareLaunchArgument("use_camera", default_value="true"),
        # 手首点群 → OctoMap。move_group と realsense の CPU を食うので必要時のみ
        DeclareLaunchArgument("use_octomap", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        # piper_single_ctrl の publish 周期と ros2_control の update_rate [Hz]。
        # 元はどちらも 200 だが N95 では CPU を食いすぎるので 100
        DeclareLaunchArgument("control_rate", default_value="100"),
        # 手首カメラの取り付け位置: camera_parent → camera_link
        # TODO: 実測値に置き換えること (今は仮の値)
        DeclareLaunchArgument("camera_parent", default_value="link6"),
        DeclareLaunchArgument("cam_x", default_value="0.0"),
        DeclareLaunchArgument("cam_y", default_value="0.0"),
        DeclareLaunchArgument("cam_z", default_value="0.0"),
        DeclareLaunchArgument("cam_roll", default_value="0.0"),
        DeclareLaunchArgument("cam_pitch", default_value="-1.5708"),
        DeclareLaunchArgument("cam_yaw", default_value="0.0"),
        # 実機の link6 (フランジ) が link5 に対してモデルからずれて付いている角度 [deg]
        # (joint6 軸まわり)。モデルの joint6 = 実機の joint6 + joint6_offset_deg
        DeclareLaunchArgument("joint6_offset_deg", default_value="0.0"),
    ]

    def make_nodes(context):

        def arg(name):
            return LaunchConfiguration(name).perform(context)

        use_octomap = arg("use_octomap").lower() == "true"
        control_rate = int(arg("control_rate"))

        robot_description = {
            "robot_description": xacro.process_file(
                xacro_file,
                mappings={
                    "joint_states_topic": "/piper_joint_bridge/joint_states",
                    "joint_commands_topic": "/piper_joint_bridge/joint_command",
                },
            ).toxml()
        }

        builder = MoveItConfigsBuilder(
            "piper", package_name="piper_with_gripper_moveit"
        )
        if use_octomap:
            builder = builder.sensors_3d(sensors_yaml)
        moveit_config = builder.to_moveit_configs()
        if not use_octomap:
            moveit_config.sensors_3d = {}

        # 実機なので use_sim_time は false
        # (piper_with_gripper_moveit/launch/piper_moveit.launch.py は true 固定)
        move_group = Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[
                moveit_config.to_dict(),
                {
                    "publish_robot_description_semantic": True,
                    "allow_trajectory_execution": True,
                    "publish_planning_scene": True,
                    "publish_geometry_updates": True,
                    "publish_state_updates": True,
                    "publish_transforms_updates": True,
                    "monitor_dynamics": False,
                    "octomap_frame": "base_link",
                    "octomap_resolution": 0.02,
                    "use_sim_time": False,
                },
            ],
        )

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
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        )

        # piper_ros の実機ノード。piper_isaacsim で Isaac Sim が担っていた部分
        piper = Node(
            package="piper",
            executable="piper_single_ctrl",
            name="piper_ctrl_single_node",
            output="screen",
            # joint_callback が受信ごとに全関節を info で出すので抑える
            ros_arguments=["--log-level", "warn"],
            parameters=[{
                "can_port": arg("can_port"),
                "auto_enable": True,
                "gripper_exist": True,
                "gripper_val_mutiple": 1,
                "publish_rate": control_rate,
            }],
        )

        bridge = Node(
            package="piper_real",
            executable="piper_joint_bridge",
            parameters=[{
                "joint_offsets": [
                    0.0, 0.0, 0.0, 0.0, 0.0,
                    math.radians(float(arg("joint6_offset_deg"))),
                ],
            }],
            output="screen",
        )

        robot_state_publisher = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[robot_description],
            output="screen",
        )

        ros2_control_node = Node(
            package="controller_manager",
            executable="ros2_control_node",
            # controllers_yaml (piper_isaacsim と共用) の update_rate を上書き
            parameters=[robot_description, controllers_yaml, arm_controller_real_yaml,
                        {"update_rate": control_rate}],
            output="screen",
        )

        def spawner(name):
            return Node(
                package="controller_manager",
                executable="spawner",
                arguments=[name, "--controller-manager-timeout", "60"],
                output="screen",
            )

        load_joint_state_broadcaster = spawner("joint_state_broadcaster")

        camera_tf = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=[
                "--x", arg("cam_x"),
                "--y", arg("cam_y"),
                "--z", arg("cam_z"),
                "--roll", arg("cam_roll"),
                "--pitch", arg("cam_pitch"),
                "--yaw", arg("cam_yaw"),
                "--frame-id", arg("camera_parent"),
                "--child-frame-id", "camera_link",
            ],
            condition=IfCondition(LaunchConfiguration("use_camera")),
        )

        if arg("use_camera").lower() != "true":
            camera = []
        else:
            camera = [IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        FindPackageShare(package="realsense2_camera").find("realsense2_camera"),
                        "launch",
                        "rs_launch.py",
                    )
                ),
                launch_arguments={
                    # yolo の3D化には color に揃えた深度が必要
                    "align_depth.enable": "true",
                    "pointcloud.enable": "true" if use_octomap else "false",
                    "rgb_camera.color_profile": "640,480,15",
                    "depth_module.depth_profile": "640,480,15",
                }.items(),
            )]

        return [
            piper,
            bridge,
            robot_state_publisher,
            ros2_control_node,
            # piper のフィードバックが ros2_control に入ってから
            # コントローラを有効化する (有効化時の関節角が保持目標になるため)
            TimerAction(period=5.0, actions=[load_joint_state_broadcaster]),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=load_joint_state_broadcaster,
                    on_exit=[
                        spawner("arm_controller"),
                        spawner("gripper_controller"),
                        spawner("gripper8_controller"),
                    ],
                )
            ),
            move_group,
            rviz,
            camera_tf,
        ] + camera

    return LaunchDescription(args + [OpaqueFunction(function=make_nodes)])
