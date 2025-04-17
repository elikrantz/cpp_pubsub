from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    joy_params = os.path.join(get_package_share_directory('ros_phoenix'),'config','joystick.yaml')

    return LaunchDescription([
        Node(
            package='cpp_pubsub',
            executable='commander',
        ),
        Node(
            package='joy',
            executable='joy_node',
            parameters=[joy_params],
        )
    ])