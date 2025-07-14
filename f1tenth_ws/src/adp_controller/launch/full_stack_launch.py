from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    utils_script = os.path.join(
        os.getenv('HOME'),
        'Desktop', 'f1tenth_ws', 'src', 'adp_controller', 'scripts', 'utils', 'raceline_publisher.py'
    )

    return LaunchDescription([
        # Frenet Error Publisher
        Node(
            package='adp_controller',
            executable='frenet_error_node',
            name='frenet_error_node',
            output='screen'
        ),

        # ADP Controller Node
        Node(
            package='adp_controller',
            executable='adp_raceline_control',
            name='adp_raceline_control',
            output='screen'
        ),

        # Raceline Publisher (for RViz path)
        ExecuteProcess(
            cmd=['python3', utils_script],
            output='screen'
        ),

        # Optional: Keyboard teleop (can be removed if driving autonomously)
        # Node(
        #     package='key_teleop',
        #     executable='key_teleop',
        #     name='key_teleop',
        #     output='screen'
        # ),
    ])

