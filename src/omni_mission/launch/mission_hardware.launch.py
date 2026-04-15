"""
mission_hardware.launch.py
===========================
Gerçek robot – RoboClaw donanım sürücüsü + LiDAR + görev düğümü.

Başlatır:
  1. omni_robot_sim/roboclaw_hw     →  RoboClaw motor sürücüsü köprüsü
                                       (/cmd_vel → motor, enkoder → /odom + TF)
  2. sllidar_ros2/sllidar_node      →  RPLiDAR A1/A2/S1 sürücüsü (/scan)
                                       (sllidar_ros2 paketi kurulu olmalı)
  3. omni_mission/mission_hw        →  Görev düğümü (use_sim_time=False)

KURULUM GEREKSİNİMLERİ:
  sudo apt install ros-jazzy-sllidar-ros2   # RPLiDAR için
  # Farklı LiDAR kullanıyorsanız bu düğümü değiştirin.

DONANIM BAĞLANTILARI:
  RoboClaw : /dev/ttyUSB0  (veya /dev/ttyS0)
  RPLiDAR  : /dev/ttyUSB1  (veya uygun port)
  Her ikisi için de udev kuralı veya chmod 666 gerekli olabilir.

Kullanım:
  ros2 launch omni_mission mission_hardware.launch.py
  ros2 launch omni_mission mission_hardware.launch.py lidar_port:=/dev/ttyUSB1
  ros2 launch omni_mission mission_hardware.launch.py roboclaw_port:=/dev/ttyUSB0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ── Argümanlar ─────────────────────────────────────────────────────────
    roboclaw_port_arg = DeclareLaunchArgument(
        'roboclaw_port',
        default_value='/dev/ttyUSB0',
        description='RoboClaw seri port'
    )
    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port',
        default_value='/dev/ttyUSB1',
        description='RPLiDAR seri port (sllidar_ros2)'
    )
    lidar_baudrate_arg = DeclareLaunchArgument(
        'lidar_baudrate',
        default_value='115200',
        description='RPLiDAR baud hizi (A1/A2=115200, S1=256000)'
    )

    # ── RoboClaw donanım sürücüsü ──────────────────────────────────────────
    # omni_robot_sim paketindeki roboclaw_hw_node'u kullanır.
    # Bu düğüm: /cmd_vel → RoboClaw → motorlar
    #           Enkoder  → /odom + TF odom→base_link
    roboclaw_node = Node(
        package='omni_robot_sim',
        executable='roboclaw_hw',
        name='roboclaw_hw_node',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'port': LaunchConfiguration('roboclaw_port'),
        }],
    )

    # ── RPLiDAR sürücüsü ──────────────────────────────────────────────────
    # sllidar_ros2 paketi kurulu değilse bu bloğu yoruma alın
    # ve LiDAR sürücünüzü ayrı terminalde elle başlatın:
    #   ros2 run sllidar_ros2 sllidar_node --ros-args -p serial_port:=/dev/ttyUSB1
    lidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='lidar_node',
        output='screen',
        parameters=[{
            'serial_port':     LaunchConfiguration('lidar_port'),
            'serial_baudrate': LaunchConfiguration('lidar_baudrate'),
            'frame_id':        'lidar_link',
            'inverted':        False,
            'angle_compensate': True,
        }],
    )

    # ── Görev düğümü (roboclaw ve lidar'ın hazır olması için 3 sn gecikme) ─
    mission_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='omni_mission',
                executable='mission_hw',
                name='omni_mission_hw',
                output='screen',
                parameters=[{'use_sim_time': False}],
            )
        ]
    )

    return LaunchDescription([
        roboclaw_port_arg,
        lidar_port_arg,
        lidar_baudrate_arg,
        roboclaw_node,
        lidar_node,
        mission_node,
    ])
