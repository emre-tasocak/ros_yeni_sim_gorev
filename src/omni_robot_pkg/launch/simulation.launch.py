"""
Gazebo Harmonic Simülasyon Launch Dosyası

Mimari (ros2_control YOK — Gazebo native plugin'ler kullanılır):
  Gazebo VelocityControl  ← ros_gz_bridge ← ROS /cmd_vel
  Gazebo OdometryPublisher → ros_gz_bridge → ROS /odom → sim_odom_tf_node → TF
  Gazebo gpu_lidar         → ros_gz_bridge → ROS /scan  → lidar_processor

Düğüm başlatma sırası:
  t= 0s : Gazebo + robot_state_publisher + RViz
  t= 5s : spawn_robot + ros_gz_bridge
  t= 9s : sim_odom_tf + lidar_processor + obstacle_avoidance + navigation + mission

Kullanım:
  ros2 launch omni_robot_pkg simulation.launch.py
  ros2 launch omni_robot_pkg simulation.launch.py gui:=false rviz:=false

Görevi başlatmak için (9s sonra):
  ros2 service call /start_mission std_srvs/srv/Trigger
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

    # --- Launch argümanları ---
    gui_arg  = DeclareLaunchArgument('gui',  default_value='true',
                                     description='Gazebo GUI açılsın mı?')
    rviz_arg = DeclareLaunchArgument('rviz', default_value='true',
                                     description='RViz açılsın mı?')
    gui  = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')

    # robot_description stringi — ParameterValue ile YAML parse hatası önlenir
    robot_description = ParameterValue(Command(['xacro ', urdf_file]), value_type=str)

    # ================================================================
    # t = 0s: Hemen başlayanlar
    # ================================================================

    # Gazebo Harmonic — GUI modu
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

    # Gazebo Harmonic — başsız (headless) mod
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

    # Robot State Publisher — hemen başlar; RViz'e robot modeli ve
    # base_footprint→base_link→laser/kamera TF'lerini sağlar
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

    # RViz (isteğe bağlı)
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
    # t = 5s: Gazebo hazır → Robot spawn + köprü
    # ================================================================

    # Robotu Gazebo'ya spawn et
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_omni_robot',
        arguments=[
            '-name',  'omni_robot',
            '-topic', 'robot_description',
            '-x', '0.0', '-y', '0.0', '-z', '0.15',  # tekerleri yerden biraz yukarı
            '-R', '0.0', '-P', '0.0', '-Y', '0.0',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # ros_gz_bridge köprüsü:
    #   /cmd_vel          : ROS Twist  → Gz  (robot hareket komutu)
    #   /model/.../odom   : Gz Odometry → ROS (odometri)
    #   /lidar_gz         : Gz LaserScan → ROS /scan
    #   /clock            : Gz Clock → ROS  (simülasyon zamanı)
    #
    # Yön sembolleri:
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
    # t = 9s: Spawn + bridge hazır → Uygulama düğümleri
    # ================================================================

    # /odom → odom→base_footprint TF yayıncısı
    # (OdometryPublisher'ın TF yayını kapalı; bu düğüm TF'yi üstlenir)
    sim_odom_tf = Node(
        package='omni_robot_pkg',
        executable='sim_odom_tf',
        name='sim_odom_tf',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # LiDAR işleme: 30cm filtre, en uzak nokta, engel tespiti
    lidar_processor = Node(
        package='omni_robot_pkg',
        executable='lidar_processor',
        name='lidar_processor',
        parameters=[params, {'use_sim_time': True}],
        output='screen',
    )

    # DWA engel kaçınma: /cmd_vel_nav → /cmd_vel (güvenli hız)
    obstacle_avoidance = Node(
        package='omni_robot_pkg',
        executable='obstacle_avoidance',
        name='obstacle_avoidance',
        parameters=[params, {'use_sim_time': True}],
        output='screen',
    )

    # Navigasyon: hedefe P-kontrolcü ile git
    navigation = Node(
        package='omni_robot_pkg',
        executable='navigation_node',
        name='navigation_node',
        parameters=[params, {'use_sim_time': True}],
        output='screen',
    )

    # Görev kontrol: durum makinesi (IDLE→SCANNING→...→DONE)
    mission = Node(
        package='omni_robot_pkg',
        executable='mission_node',
        name='mission_node',
        parameters=[params, {'use_sim_time': True}],
        output='screen',
    )

    delayed_app = TimerAction(period=9.0, actions=[
        sim_odom_tf,
        lidar_processor,
        obstacle_avoidance,
        navigation,
        mission,
    ])

    return LaunchDescription([
        gui_arg,
        rviz_arg,
        gz_sim_gui,
        gz_sim_headless,
        robot_state_publisher,
        rviz_node,
        delayed_spawn,
        delayed_app,
    ])
