#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare launch argument for robot namespace
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='',
        description='Robot namespace for GPS topics'
    )
    
    # SMARC GPS Converter Node
    smarc_gps_converter_node = Node(
        package='smarc_gps_converters',
        executable='smarc_gps_converter',
        name='smarc_gps_converter',
        namespace=LaunchConfiguration('robot_name'),
        output='both',
        remappings=[
            # Remap GPS topics to match ublox GPS node output
            ('gps_fix', 'ublox_gps_node/fix'),
            ('gps_fix_velocity', 'ublox_gps_node/fix_velocity'),
        ]
    )

    return LaunchDescription([
        robot_name_arg,
        smarc_gps_converter_node,
    ])