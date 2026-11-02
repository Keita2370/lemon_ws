"""Bringup for driving the Piper arm inside Isaac Sim from MoveIt, the same
way piper_gazebo/launch/piper_with_gripper/piper_gazebo.launch.py does for
Gazebo -- just swapping "start Gazebo + spawn_entity" for "attach to Isaac
Sim's already-running piper/joint_states + piper/joint_command topics" via
topic_based_ros2_control.

Precondition: Isaac Sim must already be running before/while this launch
runs -- go2_in_isaacsim, "Go2 with Mid-360 + Piper" robot preset, ROS2 Bridge
enabled in Preferences, and Play pressed. There's no "spawn into Isaac Sim"
step here (unlike Gazebo's spawn_entity.py) -- the articulation already
exists in the running Isaac Sim stage.
"""
import os
import re

import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def remove_comments(text):
    return re.sub(r"<!--(.*?)-->", "", text, flags=re.DOTALL)


def generate_launch_description():
    package_name = "piper_isaacsim"
    urdf_name = "piper_isaacsim.urdf.xacro"

    pkg_share = FindPackageShare(package=package_name).find(package_name)
    controllers_yaml = os.path.join(pkg_share, "config", "ros2_controllers.yaml")

    # Matches go2_in_isaacsim's settings.py DEFAULTS ("piper_joint_states_topic"
    # / "piper_joint_command_topic") -- override if those were changed in the
    # extension's Preferences (e.g. a non-empty ros2_namespace).
    joint_states_topic_arg = DeclareLaunchArgument(
        "joint_states_topic", default_value="/piper/joint_states"
    )
    joint_commands_topic_arg = DeclareLaunchArgument(
        "joint_commands_topic", default_value="/piper/joint_command"
    )

    def build_robot_description(context):
        xacro_file = os.path.join(pkg_share, "config", urdf_name)
        doc = xacro.process_file(
            xacro_file,
            mappings={
                "joint_states_topic": LaunchConfiguration("joint_states_topic").perform(context),
                "joint_commands_topic": LaunchConfiguration("joint_commands_topic").perform(context),
            },
        )
        return remove_comments(doc.toxml())

    def make_nodes(context):
        robot_description = {"robot_description": build_robot_description(context)}

        node_robot_state_publisher = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"use_sim_time": True}, robot_description],
            output="screen",
        )

        # controller_manager: loads topic_based_ros2_control against the
        # robot_description above, bridging to Isaac Sim's joint topics.
        ros2_control_node = Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[robot_description, controllers_yaml, {"use_sim_time": True}],
            output="screen",
        )

        # spawner retries until controller_manager is up, same reasoning as
        # piper_gazebo's launch (no fixed startup-order race).
        load_joint_state_broadcaster = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager-timeout", "60"],
            output="screen",
        )
        load_arm_controller = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["arm_controller", "--controller-manager-timeout", "60"],
            output="screen",
        )
        load_gripper_controller = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["gripper_controller", "--controller-manager-timeout", "60"],
            output="screen",
        )
        load_gripper8_controller = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["gripper8_controller", "--controller-manager-timeout", "60"],
            output="screen",
        )

        # joint8 has no true mimic joint in the URDF -- mirrored the same way
        # piper_gazebo does it for real/Gazebo hardware: a small node republishes
        # -joint7 onto gripper8_controller from gripper_controller's own state.
        node_gripper_mirror_controller = Node(
            package="piper_gazebo",
            executable="joint8_ctrl.py",
            output="screen",
        )

        move_group_and_rviz = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    FindPackageShare(package="piper_with_gripper_moveit").find("piper_with_gripper_moveit"),
                    "launch",
                    "piper_moveit.launch.py",
                )
            )
        )

        # ros2_control_node runs indefinitely (it's our own controller_manager,
        # not something Isaac Sim spawns and exits like Gazebo's spawn_entity),
        # so there's no OnProcessExit signal to key the first spawner off --
        # `spawner` itself already waits/retries for controller_manager to come
        # up (--controller-manager-timeout), same as piper_gazebo's launch relies
        # on. Only the second wave (the actual trajectory controllers) is chained
        # off the joint_state_broadcaster spawner's exit, same ordering reason
        # piper_gazebo uses.
        return [
            node_robot_state_publisher,
            ros2_control_node,
            node_gripper_mirror_controller,
            load_joint_state_broadcaster,
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=load_joint_state_broadcaster,
                    on_exit=[load_arm_controller, load_gripper_controller, load_gripper8_controller],
                )
            ),
            move_group_and_rviz,
        ]

    return LaunchDescription(
        [
            joint_states_topic_arg,
            joint_commands_topic_arg,
            OpaqueFunction(function=make_nodes),
        ]
    )
