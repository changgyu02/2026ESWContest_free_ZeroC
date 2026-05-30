from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # ===== 상태머신 (중앙집중 FSM) =====
        Node(
            package='core_pkg',
            executable='mission_node',
            name='mission_node',
            output='screen',
            parameters=[{
                'obstacle_wait_s': 3.0,
                'replan_delay_s':  0.5,
            }]
        ),
    ])
