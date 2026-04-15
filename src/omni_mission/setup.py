import os
from glob import glob
from setuptools import setup

package_name = 'omni_mission'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='emre',
    maintainer_email='tasocak131@gmail.com',
    description='Omni robot mission – sim and hardware versions',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Simülasyon görevi
            'mission_sim = omni_mission.mission_sim:main',
            # Gerçek robot görevi
            'mission_hw  = omni_mission.mission_hardware:main',
        ],
    },
)
