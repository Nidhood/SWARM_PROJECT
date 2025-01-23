#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# ---------------------------- PYTHON IMPORTS ----------------------------
import random

# -------------------------- LAUNCH DEPENDENCIES -------------------------
from launch_ros.actions import Node
from launch import LaunchDescription

# ----------------------------- LAUNCH SCRIPT ----------------------------
def generate_launch_description():

    # Assign the entity name:
    entity_name = "drone-" + str(int(random.random()*100000))

    # Define the starting position of the drone:
    position = [0.0, 0.0, 0.16]

    # Define the orientation of the drone:
    orientation = [0.0, 0.0, 0.0]

    # Spawn the drone model in the Gazebo simulation:
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_entity',
        output='screen',
        arguments=[
            '-entity', entity_name,
            '-topic', 'robot_description',
            '-x', str(position[0]), '-y', str(position[1]), '-z', str(position[2]),
            '-R', str(orientation[0]), '-P', str(orientation[1]), '-Y', str(orientation[2]),
        ]
    )

    # Return the launch description
    return LaunchDescription([
        spawn_robot,
    ])