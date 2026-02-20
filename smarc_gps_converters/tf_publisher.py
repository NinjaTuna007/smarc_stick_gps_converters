#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from geographic_msgs.msg import GeoPoint
import tf2_ros
from builtin_interfaces.msg import Time
import utm


class GpsToModemTfPublisher(Node):
    """
    Node to publish tf transforms for GPS and modem positions.
    1. Publishes transform from world frame to GPS position (based on GPS data in UTM coordinates)
    2. Publishes static transform from GPS position to modem position
    
    Uses UTM coordinates to match the coordinate system used by the SBG navigation driver.
    Assumes the modem is 2 meters directly below the GPS fix.
    """
    
    def __init__(self):
        super().__init__('gps_to_modem_tf_publisher')
        
        # Declare parameters
        self.declare_parameter('world_frame', 'map')  # same map frame as SBG driver
        self.declare_parameter('gps_frame', 'stick_gps')
        self.declare_parameter('modem_frame', 'stick_modem')
        self.declare_parameter('z_offset', -2.0)  # modem is 2m below GPS
        self.declare_parameter('x_offset', 0.0)
        self.declare_parameter('y_offset', 0.0)
        self.declare_parameter('publish_rate', 10.0)  # Hz
        self.declare_parameter('gps_topic', 'smarc/latlon')
        self.declare_parameter('utm_zone', 33)  # UTM Zone for Sweden
        self.declare_parameter('utm_letter', 'N')  # Northern hemisphere
        self.declare_parameter('altitude_offset', 0.0)  # Offset to correct GPS altitude (default: use raw GPS altitude to match SBG driver)
        
        # Get parameters
        self.world_frame = self.get_parameter('world_frame').value
        self.gps_frame = self.get_parameter('gps_frame').value
        self.modem_frame = self.get_parameter('modem_frame').value
        self.z_offset = self.get_parameter('z_offset').value
        self.x_offset = self.get_parameter('x_offset').value
        self.y_offset = self.get_parameter('y_offset').value
        publish_rate = self.get_parameter('publish_rate').value
        gps_topic = self.get_parameter('gps_topic').value
        self.utm_zone = self.get_parameter('utm_zone').value
        self.utm_letter = self.get_parameter('utm_letter').value
        self.altitude_offset = self.get_parameter('altitude_offset').value
        
        # Create tf2 broadcasters
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        
        # Subscribe to GPS data
        self.gps_sub = self.create_subscription(
            GeoPoint,
            gps_topic,
            self.gps_callback,
            10
        )
        
        # Send static transform from GPS to modem (this doesn't change)
        self.send_static_gps_to_modem_transform()
        
        # State variables
        self.last_gps_position = None
        
        self.get_logger().info(f'Publishing transforms:')
        self.get_logger().info(f'  {self.world_frame} -> {self.gps_frame} (dynamic, based on GPS data in UTM)')
        self.get_logger().info(f'  {self.gps_frame} -> {self.modem_frame} (static)')
        self.get_logger().info(f'Transform GPS->modem: x={self.x_offset}, y={self.y_offset}, z={self.z_offset}')
        self.get_logger().info(f'Using UTM Zone {self.utm_zone}{self.utm_letter} to match SBG driver coordinate system')
        self.get_logger().info(f'GPS altitude offset: {self.altitude_offset}m (to correct ellipsoidal->orthometric height)')
    
    def gps_callback(self, msg: GeoPoint):
        """Process GPS position and publish transform from world to GPS frame"""
        self.last_gps_position = msg
        
        # Convert GPS coordinates to UTM coordinates (same as SBG driver)
        x, y, z = self.lat_lon_to_utm(msg.latitude, msg.longitude, msg.altitude)
        
        # Publish transform from world frame to GPS frame
        self.publish_world_to_gps_transform(x, y, z)
    
    def lat_lon_to_utm(self, lat, lon, altitude=0.0):
        """
        Convert latitude/longitude to UTM coordinates.
        This matches the coordinate system used by the SBG driver.
        """
        try:
            # Convert to UTM coordinates using specified zone
            easting, northing, zone_number, zone_letter = utm.from_latlon(
                lat, lon, 
                force_zone_number=self.utm_zone,
                force_zone_letter=self.utm_letter
            )
            
            # Log zone info on first conversion
            if not hasattr(self, '_utm_zone_logged'):
                self.get_logger().info(f'Using UTM Zone {zone_number}{zone_letter} (forced: {self.utm_zone}{self.utm_letter})')
                self.get_logger().info(f'UTM coordinates: E={easting:.2f}, N={northing:.2f}')
                self._utm_zone_logged = True
            
            return easting, northing, altitude + self.altitude_offset
            
        except Exception as e:
            self.get_logger().error(f'UTM conversion failed: {e}')
            return 0.0, 0.0, 0.0
    
    def publish_world_to_gps_transform(self, x, y, z):
        """Publish transform from world frame to GPS frame"""
        transform = TransformStamped()
        
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.world_frame
        transform.child_frame_id = self.gps_frame
        
        # Translation (GPS position in ENU coordinates)
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = z
        
        # Rotation (no rotation, GPS aligned with world frame)
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0
        
        # Send the transform
        self.tf_broadcaster.sendTransform(transform)
    
    def send_static_gps_to_modem_transform(self):
        """Send the static transform from GPS to modem (only called once)"""
        static_transform = TransformStamped()
        
        static_transform.header.stamp = self.get_clock().now().to_msg()
        static_transform.header.frame_id = self.gps_frame
        static_transform.child_frame_id = self.modem_frame
        
        # Translation (modem is below GPS)
        static_transform.transform.translation.x = self.x_offset
        static_transform.transform.translation.y = self.y_offset
        static_transform.transform.translation.z = self.z_offset
        
        # Rotation (no rotation, modem aligned with GPS)
        static_transform.transform.rotation.x = 0.0
        static_transform.transform.rotation.y = 0.0
        static_transform.transform.rotation.z = 0.0
        static_transform.transform.rotation.w = 1.0
        
        # Send the static transform
        self.static_tf_broadcaster.sendTransform(static_transform)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        tf_publisher = GpsToModemTfPublisher()
        rclpy.spin(tf_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()