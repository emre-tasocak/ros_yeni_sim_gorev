"""
hardware.launch.py
==================
Launch file for the real 3-wheeled omni robot with RoboClaw hardware.

Starts:
  1. Robot State Publisher
  2. roboclaw_hw_node  – serial hardware bridge + odometry
  3. path_planner      – quintic trajectory execution
  4. rrt_planner       – LiDAR-based RRT path planning

Usage:
  ros2 launch omni_robot_sim hardware.launch.py
  ros2 launch omni_robot_sim hardware.launch.py serial_port:=/dev/ttyUSB0
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_share = get_package_share_directory('omni_robot_sim')
    xacro_file = os.path.join(pkg_share, 'urdf', 'omni_robot.urdf.xacro')

    # ── Arguments ─────────────────────────────────────────────────────────
    serial_port_arg = DeclareLaunchArgument(
        'serial_port', default_value='/dev/ttyACM0',
        description='Serial port for RoboClaw (e.g. /dev/ttyACM0)'
    )
    baud_arg = DeclareLaunchArgument(
        'baud_rate', default_value='38400',
        description='Serial baud rate for RoboClaw'
    )

    # ── Nodes ──────────────────────────────────────────────────────────────

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', xacro_file]),
            'use_sim_time': False,
        }]
    )

    roboclaw_hw = Node(
        package='omni_robot_sim',
        executable='roboclaw_hw',
        name='roboclaw_hw',
        output='screen',
        parameters=[{
            'serial_port':  LaunchConfiguration('serial_port'),
            'baud_rate':    LaunchConfiguration('baud_rate'),
            'wheel_radius': 0.05,
            'robot_radius': 0.20,
            'max_qpps':     2000,
            'cmd_timeout':  0.5,
        }]
    )

    path_planner = Node(
        package='omni_robot_sim',
        executable='path_planner',
        name='path_planner',
        output='screen',
        parameters=[{
            'max_linear_vel':  0.4,
            'max_angular_vel': 1.5,
            'traj_time_scale': 1.0,
            'goal_tolerance':  0.05,
        }]
    )

    rrt_planner = Node(
        package='omni_robot_sim',
        executable='rrt_planner',
        name='rrt_planner',
        output='screen',
        parameters=[{
            'grid_width':      8.0,
            'grid_height':     6.0,
            'grid_resolution': 0.05,
            'rrt_step':        0.15,
            'rrt_max_iter':    3000,
            'inflate_cells':   2,
        }]
    )

    return LaunchDescription([
        serial_port_arg,
        baud_arg,
        robot_state_publisher,
        roboclaw_hw,
        path_planner,
        rrt_planner,
    ])
