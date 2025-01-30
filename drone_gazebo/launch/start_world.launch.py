#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# ---------------------------- PYTHON IMPORTS ----------------------------
import os

# -------------------------- LAUNCH DEPENDENCIES -------------------------
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory, get_package_prefix

def generate_launch_description():

    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    
    # Gazebo launch path:
    gz_launch_path = PathJoinSubstitution([
        FindPackageShare("ros_gz_sim"),
        "launch",
        "gz_sim.launch.py"
    ])
    
    # World file path:
    world_file_path = PathJoinSubstitution([
        FindPackageShare("drone_gazebo"),
        "worlds",
        "example_world.sdf"
    ])
    
    # Update all gazebo model paths:
    gazebo_models_path = os.path.join(get_package_share_directory("drone_gazebo"), "models")
    
    # Paths for model of the robot:
    pkg_drone_description = "drone_description"
    install_dir = get_package_prefix(pkg_drone_description)
    
    # Environment variables configuration:
    env_vars = [
        SetEnvironmentVariable(
            name='GAZEBO_MODEL_PATH', 
            value=(
                os.environ.get('GAZEBO_MODEL_PATH', '') + ':' + 
                install_dir + "/share" + ':' + gazebo_models_path
            )
        ),
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH', 
            value=(
                os.environ.get('GZ_SIM_RESOURCE_PATH', '') + ':' + 
                install_dir + "/share" + ':' + gazebo_models_path
            )
        ),
        SetEnvironmentVariable(
            name='GAZEBO_PLUGIN_PATH', 
            value=(
                os.environ.get('GAZEBO_PLUGIN_PATH', '') + ':' + 
                install_dir + '/lib'
            )
        )
    ]
    
    ''' 
    (Does not active the controllers):
    
    Gazebo simulation launch:
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch_path),
        launch_arguments={'gz_args': world_file_path}.items(),
    ) 
    '''

    # Gazebo simulation launch:
    gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                       'launch',
                                       'gz_sim.launch.py'])]),
            launch_arguments=[('gz_args', [' -r -v 4 empty.sdf'])])
    
    return LaunchDescription(env_vars + [
        gazebo, 
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),
    ])