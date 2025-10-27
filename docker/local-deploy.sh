#!/bin/bash

# Local Docker Build and Run Script
# This script builds and runs the container locally on your development machine

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Configuration
IMAGE_NAME="1nrobotics/shiviz_px4_nav"
CONTAINER_NAME="shiviz_nav_local"
TAG=${1:-"local"}
DRONE_ID=${2:-"1"}

echo -e "${BLUE}🚁 Local Docker Build and Run${NC}"
echo -e "Container: ${YELLOW}$CONTAINER_NAME${NC}"
echo -e "Drone ID: ${YELLOW}$DRONE_ID${NC}"
echo ""

# Function to get docker command (with or without sudo)
get_docker_cmd() {
    if groups $USER | grep -q '\bdocker\b'; then
        echo "docker"
    else
        echo "sudo docker"
    fi
}

DOCKER_CMD=$(get_docker_cmd)

# Step 1: Build image locally
echo -e "${BLUE}🔨 Step 1: Building Docker image locally...${NC}"
echo "Building for current platform..."
echo -e "${YELLOW}This will build all ROS nodes in the src/ folder:${NC}"
echo "  - px4_offboard_node"
echo "  - px4ctrl"
echo "  - utils/* (all utility packages)"
echo ""

$DOCKER_CMD build -t $IMAGE_NAME:$TAG .

# Step 1.5: Clean up dangling images
echo ""
echo -e "${BLUE}🧹 Step 1.5: Cleaning up dangling Docker images...${NC}"
$DOCKER_CMD image prune -f

# Step 2: Stop and remove existing container if it exists
echo ""
echo -e "${BLUE}� Step 2: Stopping existing container...${NC}"
$DOCKER_CMD stop $CONTAINER_NAME 2>/dev/null || true
$DOCKER_CMD rm $CONTAINER_NAME 2>/dev/null || true

# Step 3: Run new container locally
echo ""
echo -e "${BLUE}� Step 3: Starting new container locally...${NC}"
$DOCKER_CMD run -d \
    --name $CONTAINER_NAME \
    --network host \
    --privileged \
    -v /dev:/dev \
    -e ROS_MASTER_URI=http://localhost:11311 \
    -e ROS_HOSTNAME=localhost \
    -e ROS_IP=localhost \
    -e PX4_SIM_HOST=localhost \
    -e PX4_SIM_MODEL=iris \
    -e VEHICLE_ID=$DRONE_ID \
    -e DRONE_ID=$DRONE_ID \
    --restart unless-stopped \
    $IMAGE_NAME:$TAG \
    bash -c 'tail -f /dev/null'

# Step 4: Check status
echo ""
echo -e "${BLUE}📊 Step 4: Checking container status...${NC}"
$DOCKER_CMD ps --filter name=$CONTAINER_NAME

echo ""
echo -e "${GREEN}✅ Local deployment completed!${NC}"
echo ""
echo -e "${BLUE}Useful commands:${NC}"
echo "  View logs:    $DOCKER_CMD logs -f $CONTAINER_NAME"
echo "  Stop:         $DOCKER_CMD stop $CONTAINER_NAME"
echo "  Shell access: $DOCKER_CMD exec -it $CONTAINER_NAME bash"
echo "  Status:       $DOCKER_CMD ps --filter name=$CONTAINER_NAME"
echo ""
echo -e "${YELLOW}To start ROS navigation manually:${NC}"
echo "  $DOCKER_CMD exec -it $CONTAINER_NAME bash"
echo "  # Inside container: source /catkin_ws/devel/setup.bash"
echo "  # Then run: roslaunch shiviz_px4_nav px4_offboard.launch"