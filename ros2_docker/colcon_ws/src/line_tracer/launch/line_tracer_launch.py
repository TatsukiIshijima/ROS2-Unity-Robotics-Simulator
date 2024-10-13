import os.path

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

'''
ros2 launch line_tracer line_tracer_launch.py
'''


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ros_tcp_endpoint'), 'launch', 'endpoint.py')
            ),
        ),
        
        Node(
            package='line_tracer',
            executable='main',
            name='line_tracer',
            output='screen',
            parameters=[{'use_sim_time': True}]
        )
    ])
