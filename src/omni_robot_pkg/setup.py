from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'omni_robot_pkg'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # URDF files
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        # World files
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        # Configuration files
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        # RViz files
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Emre Tasocak',
    maintainer_email='tasocak131@gmail.com',
    description='3-wheeled omni robot ROS2 control package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Real robot nodes
            'roboclaw_driver  = omni_robot_pkg.roboclaw_driver_node:main',
            'lidar_node       = omni_robot_pkg.lidar_node:main',
            # Shared nodes (simulation + real robot)
            'odometry_node    = omni_robot_pkg.odometry_node:main',
            'lidar_processor  = omni_robot_pkg.lidar_processor_node:main',
            'obstacle_avoidance = omni_robot_pkg.obstacle_avoidance_node:main',
            'navigation_node  = omni_robot_pkg.navigation_node:main',
            'mission_node     = omni_robot_pkg.mission_node:main',
            # Simulation: /odom → odom→base_footprint TF broadcaster
            'sim_odom_tf          = omni_robot_pkg.sim_odom_tf_node:main',
            # Simulation: Twist → wheel commands + wheel_ticks
            'sim_cmd_vel_controller = omni_robot_pkg.sim_cmd_vel_controller_node:main',
        ],
    },
)
