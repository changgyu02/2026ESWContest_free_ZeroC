import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    map_yaml = LaunchConfiguration('map')

    return LaunchDescription([

        DeclareLaunchArgument(
            'map',
            default_value='/home/changgyu/ros2_ws/src/map.yaml',
            description='맵 파일 경로 (.yaml)'
        ),

        # ===== 1. 맵 서버 =====
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'yaml_filename': map_yaml,
                'use_sim_time': False
            }]
        ),

        # ===== 2. map_server lifecycle 활성화 =====
        TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'run', 'nav2_util', 'lifecycle_bringup', 'map_server'],
                    output='screen'
                )
            ]
        ),

        # ===== 3. amcl lifecycle 활성화 (map_server 활성화 이후) =====
        # sensor_launch.py에서 amcl 노드가 먼저 실행된 상태여야 함
        TimerAction(
            period=6.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'run', 'nav2_util', 'lifecycle_bringup', 'amcl'],
                    output='screen'
                )
            ]
        ),

        # ===== 4. 경로 계획 =====
        Node(
            package='drive_pkg',
            executable='planner_node',
            name='planner_node',
            output='screen',
            parameters=[{
                'map_yaml': '/home/changgyu/ros2_ws/src/map.yaml'
            }]
        ),

        # ===== 5. 경로 추종 =====
        Node(
            package='drive_pkg',
            executable='controller_node',
            name='controller_node',
            output='screen'
        ),

    ])
