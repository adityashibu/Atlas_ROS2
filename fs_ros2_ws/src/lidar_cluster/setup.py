from setuptools import setup
import os
from glob import glob

package_name = 'lidar_cluster'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Aditya S',
    maintainer_email='as2397@hw.ac.uk',
    description='LiDAR clustering for Formula Student Driverless',
    license='GPLv3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_cluster_node = lidar_cluster.lidar_cluster_node:main',
        ],
    },
)
