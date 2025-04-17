import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    rock_threshold = LaunchConfiguration('rock_threshold', default='1.0')
    crater_threshold = LaunchConfiguration('crater_threshold', default='-1.0')
   
    lidarLaunch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_phoenix'), 'launch'),
            '/rplidar.launch.py'
        ])
    )
    depthCamLaunch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_phoenix'), 'launch'),
            '/rs_depth_launch.py'
        ])
    )
    depthProcLaunch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('cpp_pubsub'), 'launch'),
            '/depthProc.launch.py'
        ]),
        launch_arguments={
            'rock_threshold' : rock_threshold,
            'crater_threshold': crater_threshold
        }.items()
    )
    combineScanLaunch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('cpp_pubsub'), 'launch'),
            '/integrate_scan.py'
        ])
    )
    pcCombineLaunch = Node(
        package='cpp_pubsub',
        executable='pointcloud_combiner',
    )
    rviz_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('cpp_pubsub'), 'launch'),
            '/autoRviz.launch.py'
        ])
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'rock_threshold',
            default_value='0.0',
            description='1a'),
        DeclareLaunchArgument(
            'crater_threshold',
            default_value='0.0',
            description='2a'),
        
        lidarLaunch,
        depthCamLaunch,
        depthProcLaunch,
        #combineScanLaunch,
        pcCombineLaunch,
        # rviz_node
   ])