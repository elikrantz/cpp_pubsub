from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os.path

def generate_launch_description():

    rviz_config_dir = os.path.join(
            get_package_share_directory('cpp_pubsub'),
            'rviz',
            'autoSensor_ros.rviz')

    return LaunchDescription([
        Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rvi2',
            arguments=[rviz_config_dir],
            output='screen'
        )
    ])
