#!/bin/bash

# Interactive Docker Deployment Script
# Automatically configures Docker access without requiring sudo

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

# Function to run docker commands (now without sudo after setup)
run_docker_cmd() {
    local cmd="$1"
    echo -e "${BLUE}Executing: ${YELLOW}$cmd${NC}"
    ssh "$REMOTE_HOST" "$cmd"
}

# Function to setup docker group access
setup_docker_access() {
    echo -e "${BLUE}🔧 Setting up Docker access without sudo...${NC}"
    
    # Check if user is already in docker group
    if ssh "$REMOTE_HOST" "groups | grep -q '\bdocker\b'"; then
        echo -e "${GREEN}✅ User already has Docker access${NC}"
        return 0
    fi
    
    echo -e "${YELLOW}Adding user to docker group...${NC}"
    
    # Add user to docker group
    ssh -t "$REMOTE_HOST" "sudo usermod -aG docker \$USER"
    
    # Create or update docker group membership for current session
    ssh -t "$REMOTE_HOST" "sudo sh -c 'newgrp docker <<EOF
echo \"Docker group membership updated for current session\"
EOF'"
    
    # Alternative approach: restart docker daemon and reload groups
    ssh -t "$REMOTE_HOST" "sudo systemctl restart docker 2>/dev/null || true"
    
    echo -e "${GREEN}✅ Docker access configured${NC}"
    echo -e "${YELLOW}Note: You may need to log out and back in for group changes to take full effect${NC}"
}

# Step 0: Setup Docker access
setup_docker_access

# Step 1: Copy files
echo "📤 Step 1: Copying files..."
# Create remote directory first
ssh "$REMOTE_HOST" "mkdir -p ~/shiviz_px4_nav"

rsync -av --progress \
    --exclude='.git' \
    --exclude='build' \
    --exclude='devel' \
    ./ "$REMOTE_HOST":~/shiviz_px4_nav/

echo "📦 Step 1.5: Ensuring submodules are copied..."
# Copy yaml-cpp submodule files specifically
if [ -d "thirdparty/yaml-cpp" ] && [ "$(ls -A thirdparty/yaml-cpp)" ]; then
    echo "Copying yaml-cpp submodule..."
    rsync -av --progress thirdparty/yaml-cpp/ "$REMOTE_HOST":~/shiviz_px4_nav/thirdparty/yaml-cpp/
else
    echo "⚠️  Warning: yaml-cpp submodule appears empty. Run 'git submodule update --init --recursive' locally first."
fi

# Step 2: Build image
echo -e "${BLUE}🔨 Step 2: Building Docker image...${NC}"
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
    bash -c 'tail -f /dev/null'"

# Step 5: Check status
echo -e "${BLUE}📊 Step 5: Checking status...${NC}"
run_docker_cmd "docker ps --filter name=$CONTAINER_NAME"

echo -e "${GREEN}✅ Deployment completed!${NC}"
echo ""
echo -e "${BLUE}Useful commands:${NC}"
echo "  View logs:    ssh $REMOTE_HOST 'docker logs -f $CONTAINER_NAME'"
echo "  Stop:         ssh $REMOTE_HOST 'docker stop $CONTAINER_NAME'"
echo "  Shell access: ssh $REMOTE_HOST 'docker exec -it $CONTAINER_NAME bash'"
echo "  Status:       ssh $REMOTE_HOST 'docker ps'"
echo ""
echo -e "${YELLOW}To start ROS navigation manually:${NC}"
echo "  ssh $REMOTE_HOST 'docker exec -it $CONTAINER_NAME bash'"
echo "  # Inside container: source /catkin_ws/devel/setup.bash"
echo "  # Then run: roslaunch shiviz_px4_nav px4_offboard.launch"