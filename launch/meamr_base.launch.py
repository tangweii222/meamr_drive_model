from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('meamr_drive_model'),
        'config',
        'meamr_base_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='meamr_drive_model',
            executable='meamr_base_node',
            name='meamr_base_node',
            output='screen',
            parameters=[config]
        )
    ])
