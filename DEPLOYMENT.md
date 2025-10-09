# Deployment Instructions for Compulab Device

## 🎯 Target Device
- **Host**: `compulab@10.42.0.64`
- **Method**: Build on Device (Recommended)
- **Drone ID**: 1

## 📋 Prerequisites

### On Your Local Machine:
```bash
# Ensure you have SSH access to the device
ssh compulab@10.42.0.64 "echo 'Connection test successful'"

# Ensure rsync is available (usually pre-installed on Linux)
which rsync
```

### On Compulab Device:
```bash
# Docker and Docker Compose should be installed
# If not, the deployment script will attempt to install them
```

## 🚀 Quick Deployment (Recommended Method)

### Step 1: Test Connectivity
```bash
# From your project directory
cd /home/yc/src/shiviz_px4_nav

# Test SSH connection
ssh compulab@10.42.0.64 "echo 'Device accessible'"
```

### Step 2: Setup Remote Device (First Time Only)
```bash
# Install Docker and Docker Compose on the remote device
./docker/deploy.sh --setup-remote latest compulab@10.42.0.64
```

### Step 3: Deploy with Build-on-Device
```bash
# Deploy and build directly on the Compulab device
./docker/deploy.sh latest compulab@10.42.0.64 1 --remote
```

This command will:
1. 📤 Copy all source code to `compulab@10.42.0.64:~/shiviz_px4_nav/`
2. 🔨 Build Docker image directly on the device 
3. 🚀 Start the navigation containers
4. 📊 Show container status

### Step 4: Monitor Deployment
```bash
# Check deployment status
./docker/deploy.sh --status compulab@10.42.0.64

# View real-time logs
./docker/deploy.sh --logs compulab@10.42.0.64

# Access the container shell
ssh compulab@10.42.0.64
cd ~/shiviz_px4_nav
docker-compose exec shiviz_nav bash
```

## 🛠️ Manual Deployment (Alternative Method)

If the automated script doesn't work, here's the manual process:

### Step 1: Copy Files
```bash
# Copy project files to device
rsync -avz \
    --exclude='.git' \
    --exclude='build' \
    --exclude='devel' \
    --exclude='logs' \
    --exclude='*.bag' \
    --progress \
    . compulab@10.42.0.64:~/shiviz_px4_nav/
```

### Step 2: Build and Deploy on Device
```bash
# Connect to device and build
ssh compulab@10.42.0.64

# On the device:
cd ~/shiviz_px4_nav

# Build Docker image
docker build -t 1nrobotics/shiviz_px4_nav:local .

# Start services
export DRONE_ID=1
export VEHICLE_ID=1
export IMAGE_TAG=local
docker-compose down 2>/dev/null || true
docker-compose up -d

# Check status
docker-compose ps
docker-compose logs -f shiviz_nav
```

## 🎮 Navigation Commands

Once deployed, you can control the navigation system:

### On the Compulab Device:
```bash
# Access the navigation container
docker-compose exec shiviz_nav bash

# Inside container - Navigation commands:
nav_launch      # Start navigation system
nav_takeoff     # Takeoff drone
nav_land        # Land drone
nav_mission     # Start waypoint mission  
nav_stop        # Stop current mission

# PX4 commands:
px4_arm         # Arm the vehicle
px4_disarm      # Disarm the vehicle
px4_mode_offboard   # Set OFFBOARD mode
px4_mode_manual     # Set MANUAL mode

# Monitoring commands:
nav_status      # Navigation status
px4_state       # PX4 flight controller state
px4_pose        # Current pose
px4_velocity    # Current velocity
```

### From Your Local Machine:
```bash
# Remote monitoring
ssh compulab@10.42.0.64 "cd ~/shiviz_px4_nav && docker-compose logs -f shiviz_nav"

# Remote control (examples)
ssh compulab@10.42.0.64 "cd ~/shiviz_px4_nav && docker-compose exec -T shiviz_nav nav_takeoff"
```

## 🔧 Management Commands

### Update Deployment:
```bash
# Re-deploy with latest changes
./docker/deploy.sh latest compulab@10.42.0.64 1 --remote
```

### Stop Services:
```bash
# Stop all containers
./docker/deploy.sh --stop compulab@10.42.0.64

# Or manually:
ssh compulab@10.42.0.64 "cd ~/shiviz_px4_nav && docker-compose down"
```

### View Logs:
```bash
# Real-time logs
./docker/deploy.sh --logs compulab@10.42.0.64

# Or specific service:
ssh compulab@10.42.0.64 "cd ~/shiviz_px4_nav && docker-compose logs -f shiviz_nav"
```

### Debugging:
```bash
# Check container status
ssh compulab@10.42.0.64 "cd ~/shiviz_px4_nav && docker-compose ps"

# Access container for debugging
ssh compulab@10.42.0.64 "cd ~/shiviz_px4_nav && docker-compose exec shiviz_nav bash"

# Check ROS environment
ssh compulab@10.42.0.64 "cd ~/shiviz_px4_nav && docker-compose exec shiviz_nav rosenv"

# List ROS topics
ssh compulab@10.42.0.64 "cd ~/shiviz_px4_nav && docker-compose exec shiviz_nav rostopic list"
```

## 📊 Expected Output

### Successful Deployment:
```
🚁 Shiviz PX4 Navigation Deployment
🌐 Deploying to remote host: compulab@10.42.0.64
📤 Copying project files...
🔨 Building on remote device...
Building Docker image on device...
Starting deployment...
Deployment completed!
✅ Remote deployment completed

     Name                   Command               State   Ports
----------------------------------------------------------------
shiviz_nav   /ros_entrypoint.sh bash -c ...   Up
```

### Container Status:
```
CONTAINER ID   IMAGE                                    COMMAND                  STATUS
abc123def456   1nrobotics/shiviz_px4_nav:local         "/ros_entrypoint.sh …"   Up 2 minutes
```

## ⚠️ Troubleshooting

### Common Issues:

1. **SSH Connection Failed**
   ```bash
   # Check network connectivity
   ping 10.42.0.64
   
   # Try with verbose SSH
   ssh -v compulab@10.42.0.64
   ```

2. **Docker Not Found on Device**
   ```bash
   # Run setup script
   ./docker/deploy.sh --setup-remote latest compulab@10.42.0.64
   ```

3. **Build Fails on Device**
   ```bash
   # Check device resources
   ssh compulab@10.42.0.64 "df -h && free -h"
   
   # Manual build with verbose output
   ssh compulab@10.42.0.64 "cd ~/shiviz_px4_nav && docker build --no-cache -t 1nrobotics/shiviz_px4_nav:local ."
   ```

4. **Container Won't Start**
   ```bash
   # Check logs
   ssh compulab@10.42.0.64 "cd ~/shiviz_px4_nav && docker-compose logs"
   
   # Check available ports
   ssh compulab@10.42.0.64 "netstat -tulpn | grep :11311"
   ```

## 📝 Notes

- **Build Time**: First build on device may take 10-30 minutes depending on device performance
- **Network**: Ensure device has internet access for downloading dependencies
- **Storage**: Ensure device has at least 5GB free space
- **Architecture**: The Dockerfile automatically detects and builds for the device architecture
- **Persistence**: Containers will auto-restart unless manually stopped

## 🔄 Development Workflow

For ongoing development:

1. Make changes locally
2. Deploy: `./docker/deploy.sh latest compulab@10.42.0.64 1 --remote`
3. Test on device
4. Monitor: `./docker/deploy.sh --logs compulab@10.42.0.64`
5. Repeat

The deployment script efficiently handles incremental updates by only copying changed files.