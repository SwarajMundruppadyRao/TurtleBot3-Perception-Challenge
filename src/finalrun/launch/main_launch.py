from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='finalrun',
            executable='enpm673_final_proj_main',
            name='main_node',
            output='screen',
        )
    ])
