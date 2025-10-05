# Configuration Files

This directory contains configuration files for the Shiviz PX4 Navigation package.

## Files

### waypoints.yaml

Defines waypoint sequences for autonomous navigation missions.

**Format**:
```yaml
waypoints:
  - [north, east, up, yaw_deg]  # Waypoint 1
  - [north, east, up, yaw_deg]  # Waypoint 2
  # ... more waypoints
```

**Coordinate System**: NED (North-East-Down)
- `north`: Position north in meters (positive = north)
- `east`: Position east in meters (positive = east)
- `up`: Altitude in meters (positive = up from ground)
- `yaw_deg`: Heading in degrees from north (0° = north, 90° = east, -90° = west, 180° = south)

**Example**:
```yaml
waypoints:
  - [5.0, 0.0, 1.5, 0.0]      # 5m north, 1.5m altitude, heading north
  - [5.0, 5.0, 1.5, -90.0]    # 5m north, 5m east, heading west
  - [0.0, 5.0, 1.5, 180.0]    # 5m east, heading south
```

### params.yaml

General ROS parameters for the navigation system.

**Usage**: Loaded automatically by the launch file.

### ecal.ini

Configuration for eCAL (enhanced Communication Abstraction Layer) middleware.

**Purpose**: Configures eCAL communication for vision odometry integration.

**Note**: Only required when using Vilota VK180 vision system.
