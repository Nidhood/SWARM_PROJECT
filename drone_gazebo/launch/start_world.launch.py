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
        "drone_world.sdf"
    ])
    
    # Update all gazebo model paths:
    gazebo_models_path = os.path.join(get_package_share_directory("drone_gazebo"), "models")
    
    # Paths for model of the robot:
    pkg_drone_description = "drone_description"
    pkg_drone_gazebo_plugins = "drone_gazebo_plugins"
    pkg_drone_resources = "drone_gazebo"
    install_dir_model = get_package_prefix(pkg_drone_description)
    install_dir_resources = get_package_prefix(pkg_drone_resources)
    install_dir_plugins = get_package_prefix(pkg_drone_gazebo_plugins)

    
    # Environment variables configuration:
    env_vars = [
        SetEnvironmentVariable(
            name='GAZEBO_MODEL_PATH', 
            value=(
                os.environ.get('GAZEBO_MODEL_PATH', '') + ':' + 
                install_dir_model + "/share" + ':' + gazebo_models_path
            )
        ),
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH', 
            value=(
                os.environ.get('GZ_SIM_RESOURCE_PATH', '') + ':' + 
                install_dir_model + "/share" + ':' + gazebo_models_path
            )
        ),
        SetEnvironmentVariable(
            name='GAZEBO_PLUGIN_PATH', 
            value=(
                os.environ.get('GAZEBO_PLUGIN_PATH', '') + ':' + 
                install_dir_plugins + '/lib'
            )
        )
    ]
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch_path),
        launch_arguments={'gz_args': ['-r -v 4 ', world_file_path]}.items(),
    )
    
    return LaunchDescription(env_vars + [
        gazebo, 
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),
    ])