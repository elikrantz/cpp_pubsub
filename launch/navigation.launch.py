import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    rviz_config_dir = os.path.join(
            get_package_share_directory('cpp_pubsub'),
            'rviz',
            'mapping_nav2.rviz')
   
    worldLaunch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_phoenix'), 'launch'),
            '/launch_sim.launch.py'
        ]),
        launch_arguments={
            'world' : '/home/ws/src/ros_phoenix/world/lunabotic_lab_hall.world',
        }.items()
    )
    lidarLaunch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros2_laser_scan_matcher'), 'launch'),
            '/laser2odom.launch.py'
        ])
    )
    slamLaunch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('slam_toolbox'), 'launch'),
            '/online_async_launch.py'
        ]),
        launch_arguments={
            'use_sim_time' : 'false',
        }.items()
    )
    nav2Launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('nav2_bringup'), 'launch'),
            '/navigation_launch.py'
        ])
    )
    diffBotLaunch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_phoenix'), 'launch'),
            '/diffbot.launch.py'
        ]),
        launch_arguments={
            'use_container' : 'True',
        }.items()
    )
    sensorsLaunch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('cpp_pubsub'), 'launch'),
            '/autoSensor.launch.py'
        ])
    )
    rviz_node = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rvi2',
        arguments=['-d' + rviz_config_dir],
        output='screen'
    )

    return LaunchDescription([
    	diffBotLaunch,
    	sensorsLaunch,
        lidarLaunch,
        slamLaunch,
        nav2Launch,
        rviz_node,
        #worldLaunch,
   ])
