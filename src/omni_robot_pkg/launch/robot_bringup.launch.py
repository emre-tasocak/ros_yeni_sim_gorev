"""
Real Robot Bringup Launch File

Hardware:
  - 2x RoboClaw 2x15A  (/dev/ttyAMA0)
  - YDLidar X2          (/dev/ttyUSB0)
  - Intel RealSense D435b (optional, not used)
  - Encoder → Odometry

Startup sequence (hardware first, processing second):
  t= 0s : robot_state_publisher  (needed for TF immediately)
  t= 0s : roboclaw_driver + lidar_node  (connect to hardware)
  t= 3s : odometry_node  (start reading encoders)
  t= 4s : lidar_processor, obstacle_avoidance, navigation, mission

Usage:
  ros2 launch omni_robot_pkg robot_bringup.launch.py

To start mission:
  ros2 service call /start_mission std_srvs/srv/Trigger
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg         = get_package_share_directory('omni_robot_pkg')
    params_file = os.path.join(pkg, 'config', 'robot_params.yaml')
    urdf_file   = os.path.join(pkg, 'urdf', 'omni_robot.urdf.xacro')
    rviz_cfg    = os.path.join(pkg, 'rviz', 'omni_robot.rviz')

    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='false',
        description='Open RViz? (usually false for real robot)')

    rviz = LaunchConfiguration('rviz')
    robot_description = ParameterValue(Command(['xacro ', urdf_file]), value_type=str)

    # ================================================================
    # t = 0s  — TF + hardware
    # ================================================================

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': False},
        ],
        output='screen',
    )

    # Connects to RoboClaw, configures PID, resets encoders,
    # listens to /cmd_vel and publishes /wheel_ticks
    roboclaw_driver = Node(
        package='omni_robot_pkg',
        executable='roboclaw_driver',
        name='roboclaw_driver',
        parameters=[params_file],
        output='screen',
    )

    # YDLidar X2 driver — opens serial port, starts scan, publishes /scan
    lidar_node = Node(
        package='omni_robot_pkg',
        executable='lidar_node',
        name='lidar_node',
        parameters=[params_file],
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        condition=IfCondition(rviz),
        arguments=['-d', rviz_cfg],
        output='screen',
    )

    # ================================================================
    # t = 3s  — Read encoders
    # Start reading encoders after RoboClaw is connected
    # ================================================================

    # /wheel_ticks → /odom + odom→base_link TF
    odometry_node = Node(
        package='omni_robot_pkg',
        executable='odometry_node',
        name='odometry_node',
        parameters=[params_file],
        output='screen',
    )

    delayed_odom = TimerAction(period=3.0, actions=[odometry_node])

    # ================================================================
    # t = 4s  — Processing + mission nodes
    # Start navigation and mission after odometry is ready
    # ================================================================

    # /scan → 30 cm filter → /scan/filtered, /farthest_point, /nearest_obstacle
    lidar_processor = Node(
        package='omni_robot_pkg',
        executable='lidar_processor',
        name='lidar_processor',
        parameters=[params_file],
        output='screen',
    )

    # DWA obstacle avoidance: /cmd_vel_nav → /cmd_vel (safe)
    obstacle_avoidance = Node(
        package='omni_robot_pkg',
        executable='obstacle_avoidance',
        name='obstacle_avoidance',
        parameters=[params_file],
        output='screen',
    )

    # P-controller navigation: /goal_pose → /cmd_vel_nav
    navigation = Node(
        package='omni_robot_pkg',
        executable='navigation_node',
        name='navigation_node',
        parameters=[params_file],
        output='screen',
    )

    # Mission state machine: IDLE→SCANNING→NAVIGATING→AT_TARGET→RETURNING→DONE
    mission = Node(
        package='omni_robot_pkg',
        executable='mission_node',
        name='mission_node',
        parameters=[params_file],
        output='screen',
    )

    delayed_app = TimerAction(period=4.0, actions=[
        lidar_processor,
        obstacle_avoidance,
        navigation,
        mission,
    ])

    # t = 7s — All nodes ready, auto-start mission
    auto_start_mission = TimerAction(
        period=7.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'service', 'call', '/start_mission',
                     'std_srvs/srv/Trigger'],
                output='screen',
            )
        ]
    )

    return LaunchDescription([
        rviz_arg,
        robot_state_publisher,
        roboclaw_driver,
        lidar_node,
        rviz_node,
        delayed_odom,
        delayed_app,
        auto_start_mission,
    ])
