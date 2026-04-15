"""
mission_sim.launch.py
=====================
Gazebo Harmonic simülasyonu + omni_mission görev düğümü.

Başlatır:
  1. omni_robot_sim/gazebo_sim.launch.py  →  Gazebo + köprü + sim_kinematics
  2. omni_mission/mission_sim            →  Görev düğümü (use_sim_time=True)

Kullanım:
  ros2 launch omni_mission mission_sim.launch.py
  ros2 launch omni_mission mission_sim.launch.py rviz:=true
  ros2 launch omni_mission mission_sim.launch.py x:=-2.0 y:=0.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():

    # ── Argümanlar ─────────────────────────────────────────────────────────
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='false',
        description='RViz2 gorsellestirme'
    )
    x_arg = DeclareLaunchArgument('x', default_value='-2.0',
                                   description='Robot baslangic X konumu')
    y_arg = DeclareLaunchArgument('y', default_value='0.0',
                                   description='Robot baslangic Y konumu')

    # ── Gazebo simülasyonu ─────────────────────────────────────────────────
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('omni_robot_sim'),
                'launch',
                'gazebo_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'rviz': LaunchConfiguration('rviz'),
            'x':    LaunchConfiguration('x'),
            'y':    LaunchConfiguration('y'),
        }.items(),
    )

    # ── Görev düğümü (Gazebo'nun hazır olması için 5 sn gecikme) ──────────
    mission_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='omni_mission',
                executable='mission_sim',
                name='omni_mission_sim',
                output='screen',
                parameters=[{'use_sim_time': True}],
            )
        ]
    )

    return LaunchDescription([
        rviz_arg,
        x_arg,
        y_arg,
        gazebo_launch,
        mission_node,
    ])
