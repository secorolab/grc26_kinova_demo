from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    isaac_sim_path = os.environ.get('isaac_sim_package_path', '')
    if not isaac_sim_path:
        raise EnvironmentError("isaac_sim_package_path is not set!")

    pkg = get_package_share_directory('isaacsim_grc26')
    script = os.path.join(pkg, 'scripts', 'launch_sim.py')

    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                f"{isaac_sim_path}/python.sh",
                script
            ],
            output='screen'
        )
    ])
