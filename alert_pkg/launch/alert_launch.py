from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # ===== 경고음 =====
        Node(
            package='alert_pkg',
            executable='speaker_node',
            name='speaker_node',
            output='screen',
            parameters=[{
                'alert_mp3':   '/home/changgyu/ros2_ws/src/alert.mp3',
                'arrived_mp3': '/home/changgyu/ros2_ws/src/arrived.mp3',
            }]
        ),

        # ===== 경고등 (ESP32 시리얼) =====
        Node(
            package='alert_pkg',
            executable='led_node',
            name='led_node',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyCH341USB2',
                'baud_rate':   115200,
            }]
        ),

    ])
