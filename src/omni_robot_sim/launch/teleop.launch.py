"""
teleop.launch.py
================
Quick keyboard teleoperation of the omni robot (simulation or hardware).

Usage:
  ros2 launch omni_robot_sim teleop.launch.py
  (requires teleop_twist_keyboard to be installed)
    sudo apt install ros-jazzy-teleop-twist-keyboard
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    teleop = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop',
        output='screen',
        prefix='xterm -e',     # open in separate terminal window
        parameters=[{
            'stamped': False,
        }],
        remappings=[('/cmd_vel', '/omni_drive_controller/cmd_vel_unstamped')],
    )

    return LaunchDescription([teleop])
