from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    rock_threshold = LaunchConfiguration('rock_threshold', default='0.0')
    crater_threshold = LaunchConfiguration('crater_threshold', default='-0.15')

    return LaunchDescription([
        DeclareLaunchArgument(
            'rock_threshold',
            default_value='1.0'),
        DeclareLaunchArgument(
            'crater_threshold',
            default_value='-1.0'),

        Node(
            package='cpp_pubsub',
            executable='depthProc',
            parameters=[{
                'rock_threshold' : rock_threshold,
                'crater_threshold': crater_threshold
            }]
        ),
    ])
