#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare launch arguments
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='',
        description='Robot namespace for tf frames'
    )
    
    world_frame_arg = DeclareLaunchArgument(
        'world_frame',
        default_value='map',
        description='Name of the world/reference frame'
    )
    
    gps_frame_arg = DeclareLaunchArgument(
        'gps_frame',
        default_value='gps_link',
        description='Name of the GPS frame'
    )
    
    modem_frame_arg = DeclareLaunchArgument(
        'modem_frame',
        default_value='modem_link',
        description='Name of the modem frame'
    )
    
    gps_topic_arg = DeclareLaunchArgument(
        'gps_topic',
        default_value='smarc/latlon',
        description='GPS topic to subscribe to'
    )
    
    z_offset_arg = DeclareLaunchArgument(
        'z_offset',
        default_value='-2.0',
        description='Z offset from GPS to modem (negative means below)'
    )
    
    x_offset_arg = DeclareLaunchArgument(
        'x_offset',
        default_value='0.0',
        description='X offset from GPS to modem'
    )
    
    y_offset_arg = DeclareLaunchArgument(
        'y_offset',
        default_value='0.0',
        description='Y offset from GPS to modem'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='10.0',
        description='Rate at which to publish the transform (Hz)'
    )
    
    # GPS to Modem TF Publisher Node
    gps_to_modem_tf_node = Node(
        package='smarc_gps_converters',
        executable='gps_to_modem_tf_publisher',
        name='gps_to_modem_tf_publisher',
        namespace=LaunchConfiguration('robot_name'),
        output='both',
        parameters=[
            {'world_frame': LaunchConfiguration('world_frame')},
            {'gps_frame': LaunchConfiguration('gps_frame')},
            {'modem_frame': LaunchConfiguration('modem_frame')},
            {'z_offset': LaunchConfiguration('z_offset')},
            {'x_offset': LaunchConfiguration('x_offset')},
            {'y_offset': LaunchConfiguration('y_offset')},
            {'publish_rate': LaunchConfiguration('publish_rate')},
            {'gps_topic': LaunchConfiguration('gps_topic')},
        ]
    )
    
    return LaunchDescription([
        robot_name_arg,
        world_frame_arg,
        gps_frame_arg,
        modem_frame_arg,
        gps_topic_arg,
        z_offset_arg,
        x_offset_arg,
        y_offset_arg,
        publish_rate_arg,
        gps_to_modem_tf_node,
    ])