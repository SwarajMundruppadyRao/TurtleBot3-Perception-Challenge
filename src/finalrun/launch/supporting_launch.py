from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='finalrun',
            executable='horizon',
            name='horizon_node',
            output='screen',
        ),
        Node(
            package='finalrun',
            executable='stop_sign_detection',
            name='stop_sign_detection_node',
            output='screen',
        ),
        Node(
            package='finalrun',
            executable='optical_flow',
            name='optical_flow_node',
            output='screen',
        )
    ])
