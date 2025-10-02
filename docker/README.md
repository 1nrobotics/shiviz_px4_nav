# Docker Setup for Shiviz PX4 Navigation

This Docker configuration provides a containerized environment for the Shiviz PX4 Navigation system, inspired by the [Gestelt framework](https://github.com/Temasek-Dynamics/gestelt).

## 🏗️ Architecture

### Base Configuration
- **Base Image**: `ros:noetic-ros-core-focal`
- **ROS Version**: ROS Noetic  
- **Target Platform**: AMD64/ARM64 (supports Radxa and similar SBCs)
- **Dependencies**: MAVROS, PCL, Eigen3, Geographic datasets

### Container Services
- **shiviz_nav**: Main navigation service
- **px4_sitl**: PX4 SITL simulation (optional)
- **mavproxy**: Ground control proxy (optional)
- **shiviz_dev**: Development environment with GUI support

## 🚀 Quick Start

### Prerequisites
```bash
# Install Docker and Docker Compose
sudo apt update
sudo apt install docker.io docker-compose
sudo usermod -aG docker $USER
# Log out and back in for group changes to take effect
```

### Build and Run
```bash
# Build the Docker image
docker-compose build

# Run the main navigation service
docker-compose up shiviz_nav

# For development with GUI support  
xhost +local:docker
docker-compose --profile development up shiviz_dev

# For full simulation environment
docker-compose --profile simulation up
```

## 📦 Available Profiles

### Default Profile
- **shiviz_nav**: Core navigation system

### Simulation Profile  
```bash
docker-compose --profile simulation up
```
- **shiviz_nav**: Navigation system
- **px4_sitl**: PX4 SITL with Gazebo

### Development Profile
```bash
docker-compose --profile development up shiviz_dev
```
- **shiviz_dev**: Development container with GUI support
- Volume mounts for live code editing

### Ground Control Profile
```bash  
docker-compose --profile gcs up mavproxy
```
- **mavproxy**: MAVLink proxy for GCS connections

## 🛠️ Building and Development

### Build Commands
```bash
# Build for current architecture
docker-compose build

# Build for ARM64 (for Radxa deployment)
docker buildx build --platform linux/arm64 -t 1nrobotics/shiviz_px4_nav:arm64 .

# Build and push to registry
docker-compose build --push
```

### Development Workflow
```bash
# Start development container
docker-compose --profile development up -d shiviz_dev

# Access the development container
docker exec -it shiviz_dev bash

# Inside container - rebuild after changes
catkin build

# Run navigation system
nav_launch
```

## 🎮 Container Management

### Basic Operations
```bash
# Start services in background
docker-compose up -d

# View logs
docker-compose logs -f shiviz_nav

# Stop services
docker-compose down

# Restart a specific service
docker-compose restart shiviz_nav

# Shell access
docker-compose exec shiviz_nav bash
```

### Multi-Container Operations
```bash
# Start multiple profiles
docker-compose --profile simulation --profile gcs up

# Scale services (if needed)
docker-compose up --scale shiviz_nav=2

# Remove all containers and volumes
docker-compose down -v --remove-orphans
```

## 🔧 Configuration

### Environment Variables
```bash
# In .env file or docker-compose.yml
ROS_MASTER_URI=http://localhost:11311
PX4_SIM_HOST=localhost
PX4_SIM_MODEL=iris
VEHICLE_ID=1
```

### Volume Mounts
- **Source Code**: `./src` → `/catkin_ws/src/shiviz_px4_nav/src`
- **Configuration**: `./config` → `/catkin_ws/src/shiviz_px4_nav/config`  
- **Launch Files**: `./launch` → `/catkin_ws/src/shiviz_px4_nav/launch`
- **Logs**: `./logs` → `/catkin_ws/logs`

### Network Configuration
- **Mode**: `host` (for ROS communication)
- **Ports**: 11311 (ROS Master), 14540/14557 (PX4)

## 🤖 Navigation Commands

### Inside Container
```bash
# Navigation operations
nav_launch      # Launch navigation system
nav_takeoff     # Takeoff command
nav_land        # Landing command  
nav_mission     # Start waypoint mission
nav_stop        # Stop current mission

# PX4 operations  
px4_arm         # Arm the vehicle
px4_disarm      # Disarm the vehicle
px4_mode_offboard   # Set OFFBOARD mode
px4_mode_manual     # Set MANUAL mode

# Monitoring
nav_status      # Navigation status
px4_state       # PX4 flight controller state
px4_pose        # Current pose
px4_velocity    # Current velocity
```

## 🔍 Debugging and Monitoring

### Log Access
```bash
# Container logs
docker-compose logs shiviz_nav

# ROS logs inside container
docker exec -it shiviz_nav bash
tail -f /root/.ros/log/latest/rosout.log
```

### GUI Applications
```bash
# Enable X11 forwarding (Linux)
xhost +local:docker

# Run with GUI support
docker-compose --profile development up shiviz_dev

# Inside container
rviz  # or other GUI applications
```

## 📋 Troubleshooting

### Common Issues

**Permission Issues**
```bash
# Fix device permissions
sudo chmod 666 /dev/ttyACM0  # or your serial device

# Fix X11 permissions  
xhost +local:docker
```

**ROS Communication**
```bash
# Check ROS environment
docker exec -it shiviz_nav rosenv

# Test ROS communication
docker exec -it shiviz_nav rostopic list
```

**Build Failures**
```bash
# Clean rebuild
docker-compose build --no-cache

# Check rosdep dependencies
docker exec -it shiviz_nav rosdep check --from-paths src --ignore-src
```

### Resource Requirements
- **RAM**: Minimum 4GB, Recommended 8GB+  
- **CPU**: 2+ cores recommended
- **Storage**: ~5GB for image + logs

## 🚢 Deployment

### Production Deployment
```bash
# Build optimized image
docker build --build-arg CMAKE_BUILD_TYPE=Release -t 1nrobotics/shiviz_px4_nav:latest .

# Run in production mode
docker run -d --name shiviz_nav --network host --privileged \
  -v /dev:/dev \
  -e ROS_MASTER_URI=http://192.168.1.100:11311 \
  -e VEHICLE_ID=1 \
  1nrobotics/shiviz_px4_nav:latest
```

### Hardware Deployment (Radxa/ARM64)
```bash
# Build for ARM64
docker buildx build --platform linux/arm64 -t 1nrobotics/shiviz_px4_nav:arm64 .

# Deploy to remote device
docker save 1nrobotics/shiviz_px4_nav:arm64 | ssh user@radxa "docker load"
```

## 📚 References

- [Gestelt Framework](https://github.com/Temasek-Dynamics/gestelt) - Original inspiration
- [PX4 Autopilot](https://px4.io/) - Flight controller firmware  
- [MAVROS](http://wiki.ros.org/mavros) - ROS interface to MAVLink
- [Docker & ROS Guide](https://roboticseabass.com/2021/04/21/docker-and-ros/)

## 📄 License

This Docker configuration follows the same license as the main project.