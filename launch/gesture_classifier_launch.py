from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gesture_classifier',
            executable='gesture_classifier',
            name='gesture_classifier',
            output='screen'
        )
    ])
