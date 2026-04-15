import os
from glob import glob
from setuptools import setup

package_name = 'omni_robot_sim'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'urdf'),   glob('urdf/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='emre',
    maintainer_email='tasocak131@gmail.com',
    description='3-wheeled omni robot – Gazebo simulation and ROS2 nodes',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'path_planner    = omni_robot_sim.path_planner_node:main',
            'rrt_planner     = omni_robot_sim.rrt_node:main',
            'roboclaw_hw     = omni_robot_sim.roboclaw_hw_node:main',
            'sim_kinematics  = omni_robot_sim.sim_kinematics_node:main',
        ],
    },
)
