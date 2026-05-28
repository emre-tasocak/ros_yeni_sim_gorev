"""
Gazebo Harmonic Simulation Launch File

Architecture (no ros2_control — uses Gazebo native plugins):
  Gazebo VelocityControl  ← ros_gz_bridge ← ROS /cmd_vel
  Gazebo OdometryPublisher → ros_gz_bridge → ROS /odom → sim_odom_tf_node → TF
  Gazebo gpu_lidar         → ros_gz_bridge → ROS /scan  → lidar_processor

Node startup sequence:
  t= 0s : Gazebo + robot_state_publisher + RViz
  t= 5s : spawn_robot + ros_gz_bridge
  t= 9s : sim_odom_tf + lidar_processor + obstacle_avoidance + navigation + mission

Usage:
  ros2 launch omni_robot_pkg simulation.launch.py
  ros2 launch omni_robot_pkg simulation.launch.py gui:=false rviz:=false

Mission control:
  If xterm is installed, mission_node opens in a separate xterm terminal.
  If xterm is NOT installed, start mission_node manually in a separate terminal:
    ros2 run omni_robot_pkg mission_node --ros-args -p use_sim_time:=true
  Enter target coordinates: x y phi (e.g., 1.0 2.0 0)
  LiDAR is used for obstacle detection only.
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, TimerAction, IncludeLaunchDescription
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg        = get_package_share_directory('omni_robot_pkg')
    params     = os.path.join(pkg, 'config', 'robot_params.yaml')
    urdf_file  = os.path.join(pkg, 'urdf', 'omni_robot.urdf.xacro')
    world_file = os.path.join(pkg, 'worlds', 'robot_world.world')
    rviz_cfg   = os.path.join(pkg, 'rviz', 'omni_robot.rviz')
    gz_pkg     = get_package_share_directory('ros_gz_sim')

    # --- Launch arguments ---
    gui_arg  = DeclareLaunchArgument('gui',  default_value='true',
                                     description='Open Gazebo GUI?')
    rviz_arg = DeclareLaunchArgument('rviz', default_value='true',
                                     description='Open RViz?')
    gui  = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')

    # robot_description string — ParameterValue prevents YAML parse errors
    robot_description = ParameterValue(Command(['xacro ', urdf_file]), value_type=str)

    # ================================================================
    # t = 0s: Start immediately
    # ================================================================

    # Gazebo Harmonic — GUI mode
    gz_sim_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_pkg, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r -v 2 {world_file}',
            'on_exit_shutdown': 'True',
        }.items(),
        condition=IfCondition(gui),
    )

    # Gazebo Harmonic — headless mode
    gz_sim_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_pkg, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r -v 2 -s {world_file}',
            'on_exit_shutdown': 'True',
        }.items(),
        condition=UnlessCondition(gui),
    )

    # Robot State Publisher — starts immediately; provides robot model and
    # base_footprint→base_link→laser/camera TFs to RViz
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': True},
        ],
        output='screen',
    )

    # RViz (optional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        condition=IfCondition(rviz),
        arguments=['-d', rviz_cfg],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # ================================================================
    # t = 5s: Gazebo ready → Spawn robot + bridge
    # ================================================================

    # Spawn robot into Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_omni_robot',
        arguments=[
            '-name',  'omni_robot',
            '-topic', 'robot_description',
            '-x', '0.0', '-y', '0.0', '-z', '0.15',  # slightly above ground
            '-R', '0.0', '-P', '0.0', '-Y', '0.0',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # ros_gz_bridge:
    #   /cmd_vel          : ROS Twist  → Gz  (robot motion command)
    #   /model/.../odom   : Gz Odometry → ROS (odometry)
    #   /lidar_gz         : Gz LaserScan → ROS /scan
    #   /clock            : Gz Clock → ROS  (simulation time)
    #
    # Direction symbols:
    #   ]  = ROS → Gz  (subscribe ROS, publish Gz)
    #   [  = Gz → ROS  (subscribe Gz, publish ROS)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        arguments=[
            '/model/omni_robot/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/model/omni_robot/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/model/omni_robot/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/lidar_gz@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        remappings=[
            ('/model/omni_robot/cmd_vel',       '/cmd_vel'),
            ('/model/omni_robot/odometry',       '/odom'),
            ('/model/omni_robot/joint_states',   '/joint_states'),
            ('/lidar_gz',                        '/scan'),
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    delayed_spawn = TimerAction(period=5.0, actions=[
        spawn_robot,
        bridge,
    ])

    # ================================================================
    # t = 9s: Spawn + bridge ready → Application nodes
    # ================================================================

    # /odom → odom→base_footprint TF broadcaster
    # (OdometryPublisher TF disabled; this node handles TF)
    sim_odom_tf = Node(
        package='omni_robot_pkg',
        executable='sim_odom_tf',
        name='sim_odom_tf',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # LiDAR processing: 30 cm filter, farthest point, obstacle detection
    lidar_processor = Node(
        package='omni_robot_pkg',
        executable='lidar_processor',
        name='lidar_processor',
        parameters=[params, {'use_sim_time': True}],
        output='screen',
    )

    # DWA obstacle avoidance: /cmd_vel_nav → /cmd_vel (safe velocity)
    obstacle_avoidance = Node(
        package='omni_robot_pkg',
        executable='obstacle_avoidance',
        name='obstacle_avoidance',
        parameters=[params, {'use_sim_time': True}],
        output='screen',
    )

    # Navigation: P-controller to goal
    navigation = Node(
        package='omni_robot_pkg',
        executable='navigation_node',
        name='navigation_node',
        parameters=[params, {'use_sim_time': True}],
        output='screen',
    )

    # Mission control: user-input state machine
    # Needs stdin access for user input
    has_xterm = os.path.exists('/usr/bin/xterm')

    mission = Node(
        package='omni_robot_pkg',
        executable='mission_node',
        name='mission_node',
        parameters=[params, {'use_sim_time': True}],
        output='screen',
        prefix='xterm -e' if has_xterm else '',
    )

    delayed_app_nodes = [
        sim_odom_tf,
        lidar_processor,
        obstacle_avoidance,
        navigation,
    ]

    # Only include mission in launch if xterm is available
    # Otherwise user must start it manually in a separate terminal
    if has_xterm:
        delayed_app_nodes.append(mission)

    delayed_app = TimerAction(period=9.0, actions=delayed_app_nodes)

    launch_items = [
        gui_arg,
        rviz_arg,
        gz_sim_gui,
        gz_sim_headless,
        robot_state_publisher,
        rviz_node,
        delayed_spawn,
        delayed_app,
    ]

    if not has_xterm:
        # Log instruction for the user
        from launch.actions import LogInfo
        launch_items.append(
            LogInfo(msg=[
                '\n',
                '=' * 60, '\n',
                'xterm not found. Start mission_node manually:\n',
                '  ros2 run omni_robot_pkg mission_node '
                '--ros-args -p use_sim_time:=true\n',
                '=' * 60, '\n',
            ])
        )

    return LaunchDescription(launch_items)
