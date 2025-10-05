# Quick Start Guide

Get up and running with Shiviz PX4 Navigation in minutes.

## For Simulation (PX4 SITL + Gazebo)

### Prerequisites
- Ubuntu 20.04/22.04
- Docker installed

### Steps

1. **Clone the repository**
   ```bash
   git clone --recursive https://github.com/1nrobotics/shiviz_px4_nav.git
   cd shiviz_px4_nav
   ```

2. **Start PX4 SITL simulation** (in separate terminal)
   ```bash
   # Download and start PX4 SITL
   git clone https://github.com/PX4/PX4-Autopilot.git
   cd PX4-Autopilot
   make px4_sitl gazebo
   ```

3. **Build and run navigation** (in original terminal)
   ```bash
   # Build Docker image
   ./docker/build.sh
   
   # Deploy locally
   ./docker/local-deploy.sh
   
   # Access container
   docker exec -it shiviz_nav_local bash
   
   # Inside container - start navigation
   source /catkin_ws/devel/setup.bash
   roslaunch shiviz_px4_nav shiviz_px4_nav.launch
   ```

4. **Trigger mission** (in another terminal)
   ```bash
   # Access container
   docker exec -it shiviz_nav_local bash
   
   # Send commands
   rostopic pub /user_cmd std_msgs/Byte "data: 1"  # Takeoff
   # Wait for takeoff to complete
   rostopic pub /user_cmd std_msgs/Byte "data: 2"  # Start mission
   # Wait for mission to complete
   rostopic pub /user_cmd std_msgs/Byte "data: 3"  # Land
   ```

## For Real Drone

### Prerequisites
- PX4-compatible drone with MAVROS connection
- RC transmitter configured (channels 7 and 8)
- SSH access to edge device (if deploying remotely)

### Steps

1. **Deploy to edge device**
   ```bash
   ./docker/interactive-deploy.sh local your-device@IP_ADDRESS 1
   ```

2. **Access remote container**
   ```bash
   ssh your-device@IP_ADDRESS
   docker exec -it shiviz_nav bash
   ```

3. **Start navigation with RC**
   ```bash
   source /catkin_ws/devel/setup.bash
   roslaunch shiviz_px4_nav shiviz_px4_nav.launch use_rc:=true use_sim:=false
   ```

4. **Execute mission using RC**
   - Switch Mode to POSITION
   - Channel 7 center (1500) → Takeoff
   - Channel 7 bottom (1800) → Start mission
   - Channel 8 bottom (1800) → Land (emergency/manual)

## Common Issues

### "Cannot connect to MAVROS"
- Check PX4 is running: `rostopic list | grep mavros`
- Verify connection string in launch file
- For simulation: ensure PX4 SITL is running on correct port (14557)
- For hardware: check serial device path (/dev/ttyS1 or /dev/ttyUSB0)

### "Docker permission denied"
```bash
sudo usermod -aG docker $USER
# Log out and back in, or:
newgrp docker
```

### "roslaunch: command not found"
```bash
# Make sure to source ROS
source /catkin_ws/devel/setup.bash
```

### "Waypoints not loading"
- Check waypoints.yaml syntax
- Verify file path in launch file
- Check logs for parsing errors

## Next Steps

- Customize waypoints: Edit `config/waypoints.yaml`
- Adjust parameters: Edit `config/params.yaml`
- Read full documentation: See [README.md](README.md)
- Contribute: See [CONTRIBUTING.md](CONTRIBUTING.md)

## Monitoring

### Check drone state
```bash
rostopic echo /mavros/state
```

### Check position
```bash
rostopic echo /mavros/local_position/pose
```

### Monitor navigation logs
```bash
rostopic echo /rosout | grep waypoint
```

### View all topics
```bash
rostopic list
```

For more details, see the full [README.md](README.md).
