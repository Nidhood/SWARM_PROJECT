#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# ---------------------------- PYTHON IMPORTS ----------------------------
import random

# -------------------------- LAUNCH DEPENDENCIES -------------------------
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit

# ----------------------------- LAUNCH SCRIPT ----------------------------
def generate_launch_description():

    # Path to the robot controllers configuration file:
    robot_controllers = PathJoinSubstitution([
        FindPackageShare("drone_gazebo_control"),
        "config",
        "drone_controllers.yaml"
    ])

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
            '-allow_renaming', 'true',
            '-x', str(position[0]), '-y', str(position[1]), '-z', str(position[2]),
            '-R', str(orientation[0]), '-P', str(orientation[1]), '-Y', str(orientation[2]),
        ]
    )
    
    # Controller manager node
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    # Node to spawn the effort controller:
    velocity_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
                   'drone_thrust_controller',
                   '--param-file',
                   robot_controllers,
                   ],
    )

    # Node to spawn the camera controller:
    camera_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
                   'camera_360_controller',
                   '--param-file',
                   robot_controllers,
                   ],
    )

    # Bridge between ROS and Gazebo:
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # Return the launch description
    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_robot,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[velocity_controller_spawner, camera_controller_spawner],
            )
        ),
        bridge,
        spawn_robot
    ])