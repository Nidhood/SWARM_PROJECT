#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# -------------------------- LAUNCH DEPENDENCIES -------------------------
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare


# ----------------------------- LAUNCH SCRIPT ----------------------------
def generate_launch_description():
    
    # Model robot file:
    xacro_file = PathJoinSubstitution([
        FindPackageShare("drone_description"),
        "urdf",
        "drone.urdf.xacro"
    ])

    # robot_description node launch (robot_state_publisher):
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": Command(["xacro ", xacro_file])}]
    )

    # Return the launch description:
    return LaunchDescription([
        robot_state_publisher_node
    ])