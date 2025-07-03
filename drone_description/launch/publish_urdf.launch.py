#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# -------------------------- LAUNCH DEPENDENCIES -------------------------
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue 

# ----------------------------- LAUNCH SCRIPT ----------------------------
def generate_launch_description():
    
    # Model robot file:
    xacro_file = PathJoinSubstitution([
        FindPackageShare("drone_description"),
        "models",
        "cefiro2",
        "urdf",
        "drone.urdf.xacro"
    ])

    # robot_description node launch (robot_state_publisher):
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{
            "robot_description": ParameterValue(
                Command(["xacro ", xacro_file]),
                value_type=str
            )
        }]
    )

    # Return the launch description:
    return LaunchDescription([
        robot_state_publisher_node
    ])
