# smarc_gps_converters

A ROS 2 package that converts GPS data from a u-blox GPS receiver into the SMARC message format and publishes TF transforms for GPS and modem positions.

## Overview

This package contains two ROS 2 nodes:

1. **`smarc_gps_converter`** — Subscribes to `NavSatFix` and `TwistWithCovarianceStamped` messages from a u-blox GPS node and republishes them as simplified SMARC-format topics (lat/lon, speed, heading).
2. **`gps_to_modem_tf_publisher`** — Publishes TF transforms from a world frame to the GPS position (dynamic, using UTM coordinates) and a static transform from the GPS frame to a modem frame (assumed to be directly below the GPS).

## Topics

### smarc_gps_converter

| Direction | Topic | Type | Description |
|-----------|-------|------|-------------|
| Subscribes | `ublox_gps_node/fix` | `sensor_msgs/NavSatFix` | GPS fix from u-blox driver |
| Subscribes | `ublox_gps_node/fix_velocity` | `geometry_msgs/TwistWithCovarianceStamped` | GPS velocity from u-blox driver |
| Publishes | `smarc/latlon` | `geographic_msgs/GeoPoint` | Latitude, longitude, and altitude |
| Publishes | `smarc/speed` | `std_msgs/Float32` | Speed in m/s |
| Publishes | `smarc/heading` | `std_msgs/Float32` | Heading in radians |

### gps_to_modem_tf_publisher

| Direction | Topic / TF | Type | Description |
|-----------|------------|------|-------------|
| Subscribes | `smarc/latlon` | `geographic_msgs/GeoPoint` | GPS position for dynamic TF |
| Publishes TF | `map` → `stick_gps` | Dynamic transform | GPS position in UTM coordinates |
| Publishes TF | `stick_gps` → `stick_modem` | Static transform | Modem offset from GPS |

#### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `world_frame` | `map` | World/reference frame (should match SBG driver) |
| `gps_frame` | `stick_gps` | TF frame for the GPS |
| `modem_frame` | `stick_modem` | TF frame for the modem |
| `z_offset` | `-2.0` | Vertical offset from GPS to modem (m) |
| `x_offset` | `0.0` | X offset from GPS to modem (m) |
| `y_offset` | `0.0` | Y offset from GPS to modem (m) |
| `publish_rate` | `10.0` | Publish rate in Hz |
| `gps_topic` | `smarc/latlon` | GPS topic to subscribe to |
| `utm_zone` | `33` | UTM zone number (default: Sweden) |
| `utm_letter` | `N` | UTM zone letter |
| `altitude_offset` | `0.0` | Altitude correction offset (m) |

## Dependencies

- ROS 2 (tested with Humble / Iron)
- `rclpy`
- `sensor_msgs`
- `geometry_msgs`
- `geographic_msgs`
- `std_msgs`
- `tf2_ros`
- `ublox_gps` (for the GPS driver)
- Python `utm` package

## Build

```bash
cd ~/colcon_ws
colcon build --packages-select smarc_gps_converters
source install/setup.bash
```

## Launch

**GPS converter only:**

```bash
ros2 launch smarc_gps_converters smarc_gps_converter.launch.py robot_name:=<namespace>
```

**TF publisher only:**

```bash
ros2 launch smarc_gps_converters gps_to_modem_tf.launch.py robot_name:=<namespace>
```

**Full system (u-blox driver + converter):**

```bash
ros2 launch smarc_gps_converters full_gps_system.launch robot_name:=<namespace>
```

### Bringup Script

The `scripts/stick_bringup.sh` script launches the full stick system in a tmux session, including:

| tmux Window | Name | Description |
|-------------|------|-------------|
| 0 | `gps_sensors` | u-blox GPS driver |
| 1 | `waraps_agent` | WARAPS MQTT agent |
| 2 | `mqtt_bridge` | MQTT bridge |
| 3 | `acoustic_modem` | Acoustic modem ping node |
| 4 | `gps_converter` | SMARC GPS converter |
| 5 | `gps_to_modem_tf` | GPS-to-modem TF publisher |
| 6 | `empty` | Empty window for ad-hoc use |

Configuration variables are set at the top of the script (robot name, agent type, modem parameters, etc.). To run:

```bash
./scripts/stick_bringup.sh
```

This will create (or reattach to) a tmux session called `stick_bringup`.

## License

BSD-3-Clause
