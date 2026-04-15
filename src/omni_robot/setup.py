import os
from glob import glob
from setuptools import setup

package_name = 'omni_robot'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='emre',
    maintainer_email='tasocak131@gmail.com',
    description='3-wheeled omni robot base package – kinematic controller',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'omni_controller = omni_robot.omni_controller:main',
        ],
    },
)
