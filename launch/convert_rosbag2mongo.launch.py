#!/usr/bin/env python3
# Author: Alex Mitrevski

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Note: xarm_msgs were used in the KEROL project, as a robot with xArm manipulators was used.
    # This package name should be replaced with the appropriate package for your robot
    # msg_package_names = ['data_collection_msgs', 'xarm_msgs']
    # msg_package_paths = [get_package_share_directory('data_collection_msgs'),
    #                      get_package_share_directory('xarm_msgs')]

    msg_package_names = ['data_collection_msgs']
    msg_package_paths = [get_package_share_directory('data_collection_msgs')]
    config_file_path = os.path.join(
        get_package_share_directory('rosbag2mongo'),
        'config',
        'rosbag2mongo_conversion_params.yaml'
    ) 

    return LaunchDescription([
        Node(
            package='rosbag2mongo',
            executable='rosbag2mongo',
            name='rosbag2mongo_converter',
            parameters=[config_file_path,
                        {'msg_package_names': msg_package_names,
                         'msg_package_paths': msg_package_paths}]
        )
    ])
