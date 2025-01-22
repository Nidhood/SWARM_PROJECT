#!/usr/bin/env python3

# -------------------------- LAUNCH DEPENDENCIES -------------------------
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare


# ----------------------------- LAUNCH SCRIPT ----------------------------
def generate_launch_description():
    
    # Model robot file
    xacro_file = PathJoinSubstitution([
        FindPackageShare("drone_description"),
        "urdf",
        "drone.urdf.xacro"
    ])

    # robot_description node launch (robot_state_publisher)
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": Command(["xacro ", xacro_file])}]
    )

    # rviz2 node launch with an specific configuration
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare("drone_description"),
        "rviz",
        "drone_model.rviz"
    ])
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        output="screen"
    )

    # Ejecuta el nodo joint_state_publisher_gui
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen"
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])