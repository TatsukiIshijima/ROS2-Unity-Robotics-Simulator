from launch import LaunchDescription
from launch_ros.actions import Node

'''
ros2 launch line_tracer line_tracer_launch.py
'''

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='line_tracer',
            executable='main',
            name='line_tracer',
            output='screen',
            parameters=[{'use_sim_time': True}]
        )
    ])
