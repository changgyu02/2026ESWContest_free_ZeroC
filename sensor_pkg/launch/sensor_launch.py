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

        # ===== 6. odom_publisher (TF는 EKF가 담당) =====
        Node(
            package='sensor_pkg',
            executable='odom_publisher',
            name='odom_publisher',
            output='screen',
            parameters=[{
                'wheel_radius': 0.08,
                'wheel_base':   0.19,
                'publish_tf':   False
            }]
        ),

        # ===== 7. EKF (odom + IMU 융합 → TF 발행) =====
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_node',
            output='screen',
            parameters=[
                os.path.join(
                    get_package_share_directory('sensor_pkg'),
                    'config', 'ekf_params.yaml'
                )
            ]
        ),

        # ===== 8. AMCL (맵 위 위치 추정 — lifecycle은 drive_launch에서 활성화) =====
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[
                os.path.join(
                    get_package_share_directory('drive_pkg'),
                    'config', 'amcl_params.yaml'
                )
            ]
        ),

        # ===== 9. 모터 드라이버 =====
        Node(
            package='sensor_pkg',
            executable='md_control',
            name='md_control',
            output='screen'
        ),
    ])
