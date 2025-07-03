from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
import os

def generate_launch_description():
    px4_dir = os.path.join(os.getenv('HOME'), 'swarm_ws', 'src', 'drone_px4_runtime')
    startup_script = os.path.join(px4_dir, 'etc', 'init.d-posix', 'rcS')

    return LaunchDescription([
        SetEnvironmentVariable(name='PX4_SIM_MODEL', value='cefiro1'),
        ExecuteProcess(
            cmd=[
                os.path.join(px4_dir, 'bin', 'px4'),
                '-i', '0',                         
                '-w', px4_dir,                     
                '-s', startup_script               
            ],
            cwd=px4_dir,
            output='screen'
        )
    ])
