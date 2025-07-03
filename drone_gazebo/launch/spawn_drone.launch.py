from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Paths to included launch files
    gazebo_launch = PathJoinSubstitution([
        FindPackageShare("drone_gazebo"), "launch", "start_world.launch.py"
    ])
    urdf_launch = PathJoinSubstitution([
        FindPackageShare("drone_description"), "launch", "publish_urdf.launch.py"
    ])
    spawn_launch = PathJoinSubstitution([
        FindPackageShare("drone_gazebo"), "launch", "spawn_drone_description.launch.py"
    ])
    px4_launch = PathJoinSubstitution([
        FindPackageShare("drone_px4_runtime"), "launch", "px4_launcher.launch.py"
    ])

    return LaunchDescription([
        
        # Launch Gazebo first
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
        ),

        # Publish URDF (robot_description)
        TimerAction(
            period=2.0,
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(urdf_launch)
            )]
        ),

        # Spawn the drone
        TimerAction(
            period=4.0,
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(spawn_launch)
            )]
        ),

        # Launch PX4
        TimerAction(
            period=6.0,
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(px4_launch)
            )]
        ),
    ])
