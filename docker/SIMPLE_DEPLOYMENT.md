# Simple Docker Deployment (No Docker Compose)

## 🎯 **Why Use Simple Docker Instead of Docker Compose?**

### **Advantages:**
- ✅ **No additional dependencies** - Only requires Docker
- ✅ **Simpler setup** - Single container, single command
- ✅ **Less resource usage** - No compose overhead
- ✅ **Better for edge devices** - Compulab, Radxa, Raspberry Pi
- ✅ **Easier debugging** - Direct docker commands

### **Use Simple Docker When:**
- Single container application
- Edge device deployment
- Docker Compose not available
- Resource-constrained environments

### **Use Docker Compose When:**
- Multi-container applications (app + database + simulation)
- Complex networking requirements
- Development with multiple services

## 🚀 **Quick Deployment to Compulab**

### **One-Command Deployment:**
```bash
# Build on device and deploy
./docker/simple-deploy.sh local compulab@10.42.0.64 1
```

This will:
1. Copy source code to device
2. Build Docker image on device  
3. Run container with navigation system

### **Step-by-Step Commands:**

```bash
# 1. Build on remote device
./docker/simple-deploy.sh --build-remote compulab@10.42.0.64

# 2. Deploy container
./docker/simple-deploy.sh --deploy compulab@10.42.0.64 1

# 3. Check status
./docker/simple-deploy.sh --status compulab@10.42.0.64

# 4. View logs
./docker/simple-deploy.sh --logs compulab@10.42.0.64

# 5. Access container
./docker/simple-deploy.sh --shell compulab@10.42.0.64
```

## 🛠️ **Manual Docker Commands**

### **On Remote Device:**
```bash
# SSH to device
ssh compulab@10.42.0.64

# Build image
cd ~/shiviz_px4_nav
docker build -t 1nrobotics/shiviz_px4_nav:local .

# Run container
docker run -d \
    --name shiviz_nav \
    --network host \
    --privileged \
    -v /dev:/dev \
    -e ROS_MASTER_URI=http://localhost:11311 \
    -e ROS_HOSTNAME=localhost \
    -e DRONE_ID=1 \
    --restart unless-stopped \
    1nrobotics/shiviz_px4_nav:local \
    bash -c "source /catkin_ws/devel/setup.bash && roslaunch shiviz_px4_nav px4_offboard.launch"

# Check status
docker ps
docker logs -f shiviz_nav

# Access container
docker exec -it shiviz_nav bash
```

## 🎮 **Container Management**

### **Start/Stop:**
```bash
# Stop container
./docker/simple-deploy.sh --stop compulab@10.42.0.64

# Restart (deploy again)
./docker/simple-deploy.sh --deploy compulab@10.42.0.64 1
```

### **Monitoring:**
```bash
# Real-time logs
./docker/simple-deploy.sh --logs compulab@10.42.0.64

# Status check
./docker/simple-deploy.sh --status compulab@10.42.0.64

# Resource usage
ssh compulab@10.42.0.64 "docker stats shiviz_nav"
```

### **Navigation Control:**
```bash
# Access container shell
./docker/simple-deploy.sh --shell compulab@10.42.0.64

# Inside container:
nav_takeoff     # Takeoff
nav_mission     # Start mission
nav_land        # Land
px4_arm         # Arm vehicle
nav_status      # Check status
```

## 📊 **Docker vs Docker Compose Comparison**

| Feature | Simple Docker | Docker Compose |
|---------|--------------|----------------|
| **Setup** | `docker run` | `docker-compose up` |
| **Dependencies** | Docker only | Docker + Compose |
| **Resource Usage** | Lower | Higher |
| **Multi-container** | Manual | Automatic |
| **Networking** | Host/manual | Automatic |
| **Volume Management** | Manual | Declarative |
| **Environment** | CLI flags | YAML file |
| **Scaling** | Manual | Built-in |

## 🔧 **Equivalent Commands**

### **Docker Compose → Simple Docker:**

```bash
# Docker Compose:
docker-compose up -d
docker-compose logs -f
docker-compose down
docker-compose exec shiviz_nav bash

# Simple Docker:
./docker/simple-deploy.sh --deploy compulab@10.42.0.64 1
./docker/simple-deploy.sh --logs compulab@10.42.0.64
./docker/simple-deploy.sh --stop compulab@10.42.0.64
./docker/simple-deploy.sh --shell compulab@10.42.0.64
```

## ⚡ **Performance Benefits**

### **Resource Savings:**
- **Memory**: ~50-100MB less (no compose daemon)
- **CPU**: Lower overhead
- **Storage**: No compose binary needed
- **Startup**: Faster container start

### **Edge Device Benefits:**
- Works on minimal Docker installations
- Better for ARM devices (Radxa, RPi)
- Simpler troubleshooting
- Direct container control

## 🎯 **Recommended Usage**

### **Use Simple Docker for:**
- **Production deployment** on edge devices
- **Single navigation container**
- **Resource-constrained environments**
- **When Docker Compose unavailable**

### **Keep Docker Compose for:**
- **Development environment** (multi-services)
- **Simulation setup** (gazebo + navigation)
- **CI/CD pipelines**
- **Complex networking needs**