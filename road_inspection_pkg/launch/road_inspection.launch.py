from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='road_inspection_pkg',
            executable='road_inspection_node',
            name='road_inspection_node',
            output='screen'
        )
    ])
