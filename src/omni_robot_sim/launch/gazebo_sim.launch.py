"""
gazebo_sim.launch.py
====================
Gazebo Harmonic simulation of the 3-wheeled omnidirectional robot.

Architecture (no gz_ros2_control, uses native Gazebo 8 plugins):
  ROS2 /cmd_vel  →  parameter_bridge  →  Gazebo /gz_cmd_vel
                                           │
                                    VelocityControl plugin
                                    (holonomic body motion)
                                           │
                               Gazebo OdometryPublisher
                                           │
                  parameter_bridge  ←  /model/omni_robot/odometry
                           │
                       ROS2 /odom
                           │
                  sim_kinematics_node → TF odom→base_link
                                      → /joint_states (wheel angles)

Usage:
  ros2 launch omni_robot_sim gazebo_sim.launch.py
  ros2 launch omni_robot_sim gazebo_sim.launch.py rviz:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_share = get_package_share_directory('omni_robot_sim')

    # ── Arguments ──────────────────────────────────────────────────────────
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='false',
        description='Launch RViz2 for visualisation'
    )
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_share, 'worlds', 'robocup_field.sdf'),
        description='Path to Gazebo world file'
    )
    x_arg = DeclareLaunchArgument('x', default_value='-2.0')
    y_arg = DeclareLaunchArgument('y', default_value='0.0')
    # z = r_wheel = 0.05 m  →  wheels touch ground, body floats 1 cm above
    z_arg = DeclareLaunchArgument('z', default_value='0.05')

    # ── Robot description ──────────────────────────────────────────────────
    xacro_file = os.path.join(pkg_share, 'urdf', 'omni_robot.urdf.xacro')
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }]
    )

    # ── Gazebo Harmonic ────────────────────────────────────────────────────
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_args': [LaunchConfiguration('world'), ' -r'],
        }.items(),
    )

    # ── Spawn robot ────────────────────────────────────────────────────────
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_omni_robot',
        output='screen',
        arguments=[
            '-name',  'omni_robot',
            '-topic', '/robot_description',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
        ],
    )

    # ── ROS–Gazebo bridge ──────────────────────────────────────────────────
    #
    #  Topic format:
    #    topic@ROS_type[gz_type   → Gazebo→ROS  (gz publishes, ros subscribes)
    #    topic@ROS_type]gz_type   → ROS→Gazebo  (ros publishes, gz subscribes)
    #    topic@ROS_type           → bidirectional
    #
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_ros2_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[
            # Sim clock  (Gazebo → ROS2)
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',

            # cmd_vel  (ROS2 → Gazebo VelocityControl plugin)
            '/gz_cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',

            # Wheel joint velocity commands  (ROS2 → Gazebo JointController)
            '/gz_wheel_1_vel@std_msgs/msg/Float64]gz.msgs.Double',
            '/gz_wheel_2_vel@std_msgs/msg/Float64]gz.msgs.Double',
            '/gz_wheel_3_vel@std_msgs/msg/Float64]gz.msgs.Double',

            # Odometry  (Gazebo OdometryPublisher → ROS2)
            '/model/omni_robot/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',

            # LiDAR  (Gazebo → ROS2)
            '/scan_gz@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ],
        remappings=[
            ('/model/omni_robot/odometry', '/odom'),
            ('/scan_gz',                   '/scan'),
        ],
    )

    # ── Sim kinematics node ────────────────────────────────────────────────
    # Bridges cmd_vel→Gazebo, computes wheel speeds, publishes TF + joint_states
    sim_kinematics = Node(
        package='omni_robot_sim',
        executable='sim_kinematics',
        name='sim_kinematics',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # ── RViz2 (optional) ──────────────────────────────────────────────────
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        rviz_arg,
        world_arg,
        x_arg,
        y_arg,
        z_arg,
        robot_state_publisher,
        gz_sim,
        spawn_robot,
        bridge,
        sim_kinematics,
        rviz2,
    ])
