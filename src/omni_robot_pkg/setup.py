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
        # Launch dosyaları
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # URDF dosyaları
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        # Dünya dosyaları
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        # Konfigürasyon dosyaları
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        # Rviz dosyaları
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Emre Taşocak',
    maintainer_email='tasocak131@gmail.com',
    description='3-tekerli omni robot ROS2 kontrol paketi',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Gerçek robot düğümleri
            'roboclaw_driver  = omni_robot_pkg.roboclaw_driver_node:main',
            'lidar_node       = omni_robot_pkg.lidar_node:main',
            # Ortak düğümler (simülasyon + gerçek robot)
            'odometry_node    = omni_robot_pkg.odometry_node:main',
            'lidar_processor  = omni_robot_pkg.lidar_processor_node:main',
            'obstacle_avoidance = omni_robot_pkg.obstacle_avoidance_node:main',
            'navigation_node  = omni_robot_pkg.navigation_node:main',
            'mission_node     = omni_robot_pkg.mission_node:main',
            # Simülasyon: /odom → odom→base_footprint TF yayıncısı
            'sim_odom_tf          = omni_robot_pkg.sim_odom_tf_node:main',
            # (eski) Simülasyon: Twist → teker komutları + wheel_ticks
            'sim_cmd_vel_controller = omni_robot_pkg.sim_cmd_vel_controller_node:main',
        ],
    },
)
