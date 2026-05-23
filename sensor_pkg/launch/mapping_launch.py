import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([

        # ===== 1. 라이다 드라이버 =====
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('rplidar_ros'),
                    'launch', 'rplidar_a1_launch.py'
                )
            ])
        ),

        # ===== 2. base_link → laser TF =====
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_laser',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0.1',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'laser'
            ]
        ),

        # ===== 3. base_link → imu_link TF =====
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_imu',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'imu_link'
            ]
        ),

        # ===== 4. IMU 드라이버 =====
        Node(
            package='sensor_pkg',
            executable='imu_driver',
            name='imu_driver',
            output='screen'
        ),

        # ===== 5. 엔코더 드라이버 =====
        Node(
            package='sensor_pkg',
            executable='encoder_driver',
            name='encoder_driver',
            output='screen',
            parameters=[{
                'port': '/dev/ttyCH341USB1',
                'baud_rate': 115200
            }]
        ),

        # ===== 6. odom_publisher (TF 직접 발행 - 맵핑용) =====
        Node(
            package='sensor_pkg',
            executable='odom_publisher',
            name='odom_publisher',
            output='screen',
            parameters=[{
                'wheel_radius': 0.08,
                'wheel_base':   0.19,
                'publish_tf':   True
            }]
        ),

        # ===== 7. 모터 드라이버 =====
        Node(
            package='sensor_pkg',
            executable='md_control',
            name='md_control',
            output='screen'
        ),

        # ===== 8. SLAM Toolbox =====
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('slam_toolbox'),
                    'launch', 'online_async_launch.py'
                )
            ]),
            launch_arguments={
                'slam_params_file': os.path.join(
                    get_package_share_directory('sensor_pkg'),
                    'config', 'mapper_params.yaml'
                ),
                'use_sim_time': 'false'
            }.items()
        ),

        # ===== 9. 텔레옵 =====
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop',
            output='screen',
            prefix='xterm -e'
        ),

        # ===== 10. RViz2 =====
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        ),
    ])
