import os
from setuptools import setup, find_packages
from glob import glob

package_name = 'rosbag2mongo'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alex Mitrevski',
    maintainer_email='alemitr@chalmers.se',
    description='Utilities for converting data from ROS 2 bag files to an RLDS-like MongoDB format',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rosbag2mongo = rosbag2mongo.rosbag2mongo:main'
        ],
    }
)
