import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    map_yaml = LaunchConfiguration('map')

    return LaunchDescription([

        DeclareLaunchArgument(
            'map',
            default_value='/home/changgyu/ros2_ws/src/map.yaml',
            description='맵 파일 경로 (.yaml)'
        ),

        # ===== 1. 센서 레이어 =====
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('sensor_pkg'),
                    'launch', 'sensor_launch.py'
                )
            )
        ),

        # ===== 2. 주행 레이어 (센서 기동 후 5초 대기) =====
        TimerAction(
            period=5.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(
                            get_package_share_directory('drive_pkg'),
                            'launch', 'drive_launch.py'
                        )
                    ),
                    launch_arguments={'map': map_yaml}.items()
                )
            ]
        ),

        # ===== 3. 상태머신 레이어 (주행 레이어 기동 후 8초 대기) =====
        TimerAction(
            period=13.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(
                            get_package_share_directory('core_pkg'),
                            'launch', 'core_launch.py'
                        )
                    )
                )
            ]
        ),

        # ===== 4. 경보 레이어 (상태머신 기동 후 2초 대기) =====
        TimerAction(
            period=15.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(
                            get_package_share_directory('alert_pkg'),
                            'launch', 'alert_launch.py'
                        )
                    )
                )
            ]
        ),
    ])
