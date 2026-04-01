"""
arm_bringup.launch.py
Launches servo_node, rgb_led_node, and bt_node together.
Override params on the command line, e.g.:
  ros2 launch robot_arm_bringup arm_bringup.launch.py device_name:=MyArm
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        DeclareLaunchArgument('device_name',   default_value='RobotArm'),

        # ── Web dashboard node ────────────────────────────────────────
        Node(
            package='robot_arm_bringup',
            executable='web_node.py',
            name='web_node',
            output='screen',
            parameters=[{
                'host': '0.0.0.0',
                'port': 8080,
            }],
        ),

        # ── Sound node ────────────────────────────────────────────────
        Node(
            package='robot_arm_bringup',
            executable='sound_node.py',
            name='sound_node',
            output='screen',
            parameters=[{
                'sound_init':     '/home/fishbot/sounds/init.wav',
                'sound_ready':    '/home/fishbot/sounds/ready.wav',
                'ready_nodes':    '/servo_node,/rgb_led_node,/bt_node,/web_node',
                'check_interval': 2.0,
                'aplay_device':   'default',
            }],
        ),

        # ── Servo node ────────────────────────────────────────────────
        Node(
            package='robot_arm_bringup',
            executable='servo_node.py',
            name='servo_node',
            output='screen',
            parameters=[{
                'i2c_address':     0x40,
                'pwm_frequency':   50.0,
                'pulse_min_us':    500.0,
                'pulse_max_us':    2500.0,
                'channel_base':    0,
                'channel_shoulder': 1,
                'channel_elbow':   2,
                'channel_gripper': 3,
            }],
        ),

        # ── RGB LED node (WS2812B over SPI on Ubuntu) ─────────────────
        Node(
            package='robot_arm_bringup',
            executable='rgb_led_node.py',
            name='rgb_led_node',
            output='screen',
            parameters=[{
                'spi_bus':      0,
                'spi_device':   0,
                'spi_speed_hz': 6_400_000,
                'led_count':    1,
            }],
        ),

        # ── Bluetooth node ────────────────────────────────────────────
        Node(
            package='robot_arm_bringup',
            executable='bt_node.py',
            name='bt_node',
            output='screen',
            parameters=[{
                'device_name':     LaunchConfiguration('device_name'),
                'status_interval': 1.0,
            }],
        ),

    ])
