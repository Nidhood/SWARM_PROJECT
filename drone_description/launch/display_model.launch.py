#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# -------------------------- LAUNCH DEPENDENCIES -------------------------
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare


# ----------------------------- LAUNCH SCRIPT ----------------------------
def generate_launch_description():
    
    # rviz2 node launch with an specific configuration:
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare("drone_description"),
        "rviz",
        "drone_model.rviz"
    ])

    # rviz2 node launch:
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        output="screen"
    )

    # Execute joint_state_publisher_gui node:
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen"
    )

    # Return the launch description:
    return LaunchDescription([
        joint_state_publisher_gui_node,
        rviz_node
    ])