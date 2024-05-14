import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import (
    LaunchConfiguration, 
    Command
)


def generate_launch_description():


    # Declare the launch argument for the world file:
    world = os.path.join(get_package_share_directory('drone_model'), 'worlds', 'empty_world.world')

    # Declare the launch argument for the use_sim_time parameter:
    urdf_drone_model_path = os.path.join(get_package_share_directory('drone_model'), 'urdf', 'drone_model.urdf')

    return LaunchDescription([

            # Declare the launch argument for the drone model:
            launch.actions.DeclareLaunchArgument(name='model', default_value=urdf_drone_model_path, description='Path to the drone model URDF file'),

            # Declare the launch argument for the world file:
            launch.actions.ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world], output='screen'),
            
             # Launch the joint state publisher GUI:
            Node(
                package='joint_state_publisher_gui',
                executable='joint_state_publisher_gui',
                name='joint_state_publisher_gui',
            ),


            # Publish the robot state node:
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}],
            ),
           
            # Display the drone model in Gazebo:
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', 'physics',
                    '-topic', 'robot_description',
                    '-z', '1.0',
                    '-x', '0'
                ],
                output='screen'
            ),
                           
    ])