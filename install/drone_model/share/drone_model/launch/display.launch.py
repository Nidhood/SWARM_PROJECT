import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():


    # Declare the launch argument for the use_sim_time parameter:
    urdf_drone_model_path = os.path.join(get_package_share_directory('drone_model'), 'urdf', 'drone_model.urdf')
    rviz_config_dir = os.path.join(get_package_share_directory('drone_model'), 'rviz', 'drone_model.rviz')

    return LaunchDescription([

            # Declare the launch argument for the drone model:
            launch.actions.DeclareLaunchArgument(name='model', default_value=urdf_drone_model_path, description='Path to the drone model URDF file'),

            # Launch the joint state publisher GUI:
            Node(
                package='joint_state_publisher_gui',
                executable='joint_state_publisher_gui',
                name='joint_state_publisher_gui',
                arguments=[urdf_drone_model_path],
                output='screen'
            ),
            

            # Publish the robot state node:
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}],
            ),
            

            # Display the drone model in RViz:
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config_dir],
                output='screen'),
    ])


    
