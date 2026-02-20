#!/usr/bin/env python3

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistWithCovarianceStamped

from std_msgs.msg import Float32
from geographic_msgs.msg import GeoPoint


class SmarcGpsConverter(Node):
    """
    Converts GPS NavSatFix and TwistWithCovarianceStamped messages to SMARC format.
    
    Subscribes to:
    - ~gps_fix (sensor_msgs/NavSatFix)
    - ~gps_fix_velocity (geometry_msgs/TwistWithCovarianceStamped)
    
    Publishes to:
    - ~smarc/latlon (geographic_msgs/GeoPoint) - lat, lon, altitude
    - ~smarc/speed (std_msgs/Float32) - speed in m/s
    - ~smarc/heading (std_msgs/Float32) - heading in radians
    """
    
    def __init__(self):
        super().__init__('smarc_gps_converter')
        
        # QoS profile for GPS data
        gps_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # Subscribers
        self.gps_fix_sub = self.create_subscription(
            NavSatFix,
            'ublox_gps_node/fix',
            self.gps_fix_callback,
            gps_qos
        )
        
        self.gps_velocity_sub = self.create_subscription(
            TwistWithCovarianceStamped,
            'ublox_gps_node/fix_velocity',
            self.gps_velocity_callback,
            gps_qos
        )
        
        # Publishers
        self.latlon_pub = self.create_publisher(GeoPoint, 'smarc/latlon', 10)
        self.speed_pub = self.create_publisher(Float32, 'smarc/speed', 10)
        self.heading_pub = self.create_publisher(Float32, 'smarc/heading', 10)
        
        # State variables
        self.last_fix: Optional[NavSatFix] = None
        self.last_velocity: Optional[TwistWithCovarianceStamped] = None
        
        self.get_logger().info('SMARC GPS Converter initialized')
    
    def gps_fix_callback(self, msg: NavSatFix):
        """Process GPS fix messages and publish lat/lon/altitude."""
        self.last_fix = msg
        
        # Only publish if we have a valid fix
        if msg.status.status >= 0:  # STATUS_NO_FIX = -1, so >= 0 means we have some fix
            latlon_msg = GeoPoint()
            latlon_msg.latitude = msg.latitude   # degrees
            latlon_msg.longitude = msg.longitude  # degrees
            latlon_msg.altitude = msg.altitude   # meters

            self.latlon_pub.publish(latlon_msg)
            
            self.get_logger().debug(
                f'Published latlon: lat={msg.latitude:.6f}, lon={msg.longitude:.6f}, alt={msg.altitude:.2f}'
            )
    
    def gps_velocity_callback(self, msg: TwistWithCovarianceStamped):
        """Process GPS velocity messages and publish speed and heading."""
        self.last_velocity = msg
        
        twist = msg.twist.twist
        
        # Calculate speed (magnitude of velocity vector)
        vx = twist.linear.x
        vy = twist.linear.y
        vz = twist.linear.z
        
        speed = math.sqrt(vx*vx + vy*vy + vz*vz)
        
        speed_msg = Float32()
        speed_msg.data = speed
        self.speed_pub.publish(speed_msg)
        
        # Calculate heading from velocity components
        # atan2(vy, vx) gives heading in radians (-pi to pi)
        # Note: This assumes standard mathematical convention where:
        # - positive x is east
        # - positive y is north
        # - heading is measured counterclockwise from east
        heading = math.atan2(vy, vx)
        
        heading_msg = Float32()
        heading_msg.data = heading
        self.heading_pub.publish(heading_msg)
        
        self.get_logger().debug(
            f'Published speed: {speed:.2f} m/s, heading: {heading:.3f} rad ({math.degrees(heading):.1f}°)'
        )


def main(args=None):
    rclpy.init(args=args)
    
    node = SmarcGpsConverter()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down SMARC GPS Converter')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()