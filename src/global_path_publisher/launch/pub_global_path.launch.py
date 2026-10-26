from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='global_path_publisher', executable='global_path_pub', output='screen',
             parameters=[{'start_x': -10.0, 'start_y': 0.0, 'end_x': 9.0, 'end_y': 0.0}]),
    ])
