#!/bin/bash

# Interactive Docker Deployment Script with Sudo Support
# Handles sudo password prompts properly

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Configuration
IMAGE_NAME="1nrobotics/shiviz_px4_nav"
CONTAINER_NAME="shiviz_nav"
TAG=${1:-"local"}
REMOTE_HOST=${2:-"compulab@10.42.0.64"}
DRONE_ID=${3:-"1"}

echo -e "${BLUE}🚁 Interactive Docker Deployment${NC}"
echo -e "Target: ${YELLOW}$REMOTE_HOST${NC}"
echo -e "Drone ID: ${YELLOW}$DRONE_ID${NC}"
echo ""

# Function to run docker commands with proper sudo handling
run_docker_cmd() {
    local cmd="$1"
    echo -e "${BLUE}Executing: ${YELLOW}sudo $cmd${NC}"
    ssh -t "$REMOTE_HOST" "sudo $cmd"
}

# Step 1: Copy files
echo "📤 Step 1: Copying files..."
# Create remote directory first
ssh compulab@$REMOTE_HOST "mkdir -p ~/shiviz_px4_nav"

rsync -av --progress \
    --exclude='.git' \
    --exclude='build' \
    --exclude='devel' \
    ./ compulab@$REMOTE_HOST:~/shiviz_px4_nav/

echo "📦 Step 1.5: Ensuring submodules are copied..."
# Copy yaml-cpp submodule files specifically
if [ -d "thirdparty/yaml-cpp" ] && [ "$(ls -A thirdparty/yaml-cpp)" ]; then
    echo "Copying yaml-cpp submodule..."
    rsync -av --progress thirdparty/yaml-cpp/ compulab@$REMOTE_HOST:~/shiviz_px4_nav/thirdparty/yaml-cpp/
else
    echo "⚠️  Warning: yaml-cpp submodule appears empty. Run 'git submodule update --init --recursive' locally first."
fi

# Step 2: Build image
echo -e "${BLUE}🔨 Step 2: Building Docker image...${NC}"
echo "This will require sudo password on the remote device."
run_docker_cmd "docker build -t ${IMAGE_NAME}:${TAG} ~/shiviz_px4_nav/"

# Step 3: Stop existing container
echo -e "${BLUE}🛑 Step 3: Stopping existing container...${NC}"
run_docker_cmd "docker stop $CONTAINER_NAME 2>/dev/null || true"
run_docker_cmd "docker rm $CONTAINER_NAME 2>/dev/null || true"

# Step 4: Run new container
echo -e "${BLUE}🚀 Step 4: Starting new container...${NC}"
run_docker_cmd "docker run -d \\
    --name $CONTAINER_NAME \\
    --network host \\
    --privileged \\
    -v /dev:/dev \\
    -e ROS_MASTER_URI=http://localhost:11311 \\
    -e ROS_HOSTNAME=localhost \\
    -e ROS_IP=localhost \\
    -e PX4_SIM_HOST=localhost \\
    -e PX4_SIM_MODEL=iris \\
    -e VEHICLE_ID=$DRONE_ID \\
    -e DRONE_ID=$DRONE_ID \\
    --restart unless-stopped \\
    ${IMAGE_NAME}:${TAG} \\
    bash -c 'source /catkin_ws/devel/setup.bash && roslaunch shiviz_px4_nav px4_offboard.launch'"

# Step 5: Check status
echo -e "${BLUE}📊 Step 5: Checking status...${NC}"
run_docker_cmd "docker ps --filter name=$CONTAINER_NAME"

echo -e "${GREEN}✅ Deployment completed!${NC}"
echo ""
echo -e "${BLUE}Useful commands:${NC}"
echo "  View logs:    ssh $REMOTE_HOST 'sudo docker logs -f $CONTAINER_NAME'"
echo "  Stop:         ssh $REMOTE_HOST 'sudo docker stop $CONTAINER_NAME'"
echo "  Shell access: ssh $REMOTE_HOST 'sudo docker exec -it $CONTAINER_NAME bash'"
echo "  Status:       ssh $REMOTE_HOST 'sudo docker ps'"