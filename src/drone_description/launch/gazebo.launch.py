#!/usr/bin/env python3

# -------------------------- LAUNCH DEPENDENCIES -------------------------
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare

# ----------------------------- LAUNCH SCRIPT ----------------------------
def generate_launch_description():
    
    # Paths for the robot
    xacro_file = PathJoinSubstitution([
        FindPackageShare("drone_description"),
        "urdf",
        "drone.urdf.xacro"
    ])

    # World file path
    world_file_path = PathJoinSubstitution([
        FindPackageShare("drone_description"),
        "worlds",
        "example_world.sdf"
    ])

    # Gazebo launch path
    gz_launch_path = PathJoinSubstitution([
        FindPackageShare("ros_gz_sim"),
        "launch",
        "gz_sim.launch.py"
    ])

    # Generate URDF from XACRO
    robot_description = Command(['xacro ', xacro_file])

    # Environment variables for Gazebo resources and plugins
    resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        PathJoinSubstitution([
            FindPackageShare("drone_description"),
            "meshes"
        ])
    )

    plugin_path = SetEnvironmentVariable(
        'GZ_SIM_PLUGIN_PATH',
        PathJoinSubstitution([
            FindPackageShare("drone_description"),
            "plugins"
        ])
    )

    # Gazebo simulation launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch_path),
        launch_arguments={
            'gz_args': world_file_path,
        }.items(),
    )

    # Spawn robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'swarm_drone',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '1.0',
        ],
        output='screen',
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    return LaunchDescription([
        resource_path,
        plugin_path,
        gazebo,
        robot_state_publisher,
        spawn_robot,
    ])