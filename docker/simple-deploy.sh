#!/bin/bash

# Simple Docker Deployment Script (No Docker Compose Required)
# For deploying shiviz_px4_nav to edge devices

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
REMOTE_HOST=${2:-""}
DRONE_ID=${3:-"1"}
USE_SUDO=${USE_SUDO:-"auto"}  # auto, true, false

echo -e "${BLUE}🚁 Simple Docker Deployment for Shiviz PX4 Navigation${NC}"

# Function to determine if sudo is needed for docker commands
get_docker_cmd() {
    local host_cmd="$1"
    local docker_cmd="docker"
    
    if [[ "$USE_SUDO" == "true" ]]; then
        docker_cmd="sudo docker"
    elif [[ "$USE_SUDO" == "auto" ]]; then
        # Test if docker works without sudo
        if [[ -n "$host_cmd" ]]; then
            # Remote host - test docker access
            if ! $host_cmd docker ps &>/dev/null; then
                docker_cmd="sudo docker"
            fi
        else
            # Local host - test docker access
            if ! docker ps &>/dev/null; then
                docker_cmd="sudo docker"
            fi
        fi
    fi
    
    echo "$docker_cmd"
}

# Function to show usage
show_usage() {
    echo "Usage: $0 [TAG] [REMOTE_HOST] [DRONE_ID]"
    echo ""
    echo "Arguments:"
    echo "  TAG         Docker image tag (default: local)"
    echo "  REMOTE_HOST Remote host (user@hostname) for deployment"
    echo "  DRONE_ID    Drone identifier (default: 1)"
    echo ""
    echo "Examples:"
    echo "  $0                                    # Local deployment"
    echo "  $0 local compulab@10.42.0.64 1       # Remote deployment"
    echo "  USE_SUDO=true $0 local compulab@10.42.0.64 1  # Force sudo usage"
    echo ""
    echo "Commands:"
    echo "  --build         Build image locally"
    echo "  --build-remote  Build on remote device"
    echo "  --deploy        Deploy container"
    echo "  --stop          Stop container"
    echo "  --logs          Show logs"
    echo "  --shell         Access container shell"
    echo "  --status        Show status"
    echo ""
    echo "Environment Variables:"
    echo "  USE_SUDO=auto   Auto-detect if sudo is needed (default)"
    echo "  USE_SUDO=true   Always use sudo for docker commands"
    echo "  USE_SUDO=false  Never use sudo for docker commands"
}

# Function to build image locally
build_local() {
    echo -e "${BLUE}🔨 Building Docker image locally...${NC}"
    local docker_cmd=$(get_docker_cmd "")
    $docker_cmd build -t "${IMAGE_NAME}:${TAG}" .
    echo -e "${GREEN}✅ Build completed${NC}"
}

# Function to build on remote device
build_remote() {
    if [[ -z "$REMOTE_HOST" ]]; then
        echo -e "${RED}❌ Remote host not specified${NC}"
        exit 1
    fi
    
    echo -e "${BLUE}📤 Copying files to remote device...${NC}"
    rsync -avz \
        --exclude='.git' \
        --exclude='build' \
        --exclude='devel' \
        --exclude='logs' \
        --exclude='*.bag' \
        --progress \
        . "$REMOTE_HOST:~/shiviz_px4_nav/"
    
    echo -e "${BLUE}🔨 Building on remote device...${NC}"
    local docker_cmd=$(get_docker_cmd "ssh $REMOTE_HOST")
    ssh "$REMOTE_HOST" "cd ~/shiviz_px4_nav && $docker_cmd build -t ${IMAGE_NAME}:${TAG} ."
    echo -e "${GREEN}✅ Remote build completed${NC}"
}

# Function to deploy container
deploy_container() {
    local host_cmd=""
    if [[ -n "$REMOTE_HOST" ]]; then
        host_cmd="ssh $REMOTE_HOST"
    fi
    
    local docker_cmd=$(get_docker_cmd "$host_cmd")
    
    echo -e "${BLUE}🚀 Deploying container...${NC}"
    
    # Stop existing container
    $host_cmd $docker_cmd stop "$CONTAINER_NAME" 2>/dev/null || true
    $host_cmd $docker_cmd rm "$CONTAINER_NAME" 2>/dev/null || true
    
    # Run new container
    $host_cmd $docker_cmd run -d \
        --name "$CONTAINER_NAME" \
        --network host \
        --privileged \
        -v /dev:/dev \
        -e ROS_MASTER_URI=http://localhost:11311 \
        -e ROS_HOSTNAME=localhost \
        -e ROS_IP=localhost \
        -e PX4_SIM_HOST=localhost \
        -e PX4_SIM_MODEL=iris \
        -e VEHICLE_ID="$DRONE_ID" \
        -e DRONE_ID="$DRONE_ID" \
        --restart unless-stopped \
        "${IMAGE_NAME}:${TAG}" \
        bash -c "source /catkin_ws/devel/setup.bash && roslaunch shiviz_px4_nav px4_offboard.launch"
    
    echo -e "${GREEN}✅ Container deployed${NC}"
    
    # Show status
    sleep 2
    show_status
}

# Function to stop container
stop_container() {
    local host_cmd=""
    if [[ -n "$REMOTE_HOST" ]]; then
        host_cmd="ssh $REMOTE_HOST"
    fi
    
    local docker_cmd=$(get_docker_cmd "$host_cmd")
    
    echo -e "${BLUE}🛑 Stopping container...${NC}"
    $host_cmd $docker_cmd stop "$CONTAINER_NAME" 2>/dev/null || true
    $host_cmd $docker_cmd rm "$CONTAINER_NAME" 2>/dev/null || true
    echo -e "${GREEN}✅ Container stopped${NC}"
}

# Function to show logs
show_logs() {
    local host_cmd=""
    if [[ -n "$REMOTE_HOST" ]]; then
        host_cmd="ssh $REMOTE_HOST"
    fi
    
    local docker_cmd=$(get_docker_cmd "$host_cmd")
    
    echo -e "${BLUE}📋 Container logs:${NC}"
    $host_cmd $docker_cmd logs -f "$CONTAINER_NAME"
}

# Function to access shell
access_shell() {
    local host_cmd=""
    if [[ -n "$REMOTE_HOST" ]]; then
        host_cmd="ssh $REMOTE_HOST"
    fi
    
    local docker_cmd=$(get_docker_cmd "$host_cmd")
    
    echo -e "${BLUE}🐚 Accessing container shell...${NC}"
    $host_cmd $docker_cmd exec -it "$CONTAINER_NAME" bash
}

# Function to show status
show_status() {
    local host_cmd=""
    if [[ -n "$REMOTE_HOST" ]]; then
        host_cmd="ssh $REMOTE_HOST"
        echo -e "${BLUE}📊 Status on remote host: ${YELLOW}$REMOTE_HOST${NC}"
    else
        echo -e "${BLUE}📊 Local Status${NC}"
    fi
    
    local docker_cmd=$(get_docker_cmd "$host_cmd")
    
    $host_cmd $docker_cmd ps --filter "name=$CONTAINER_NAME" --format "table {{.Names}}\t{{.Status}}\t{{.Ports}}"
    
    # Check if container is running
    if $host_cmd $docker_cmd ps --filter "name=$CONTAINER_NAME" --filter "status=running" | grep -q "$CONTAINER_NAME"; then
        echo -e "${GREEN}✅ Container is running${NC}"
    else
        echo -e "${RED}❌ Container is not running${NC}"
        echo "Recent logs:"
        $host_cmd $docker_cmd logs --tail 10 "$CONTAINER_NAME" 2>/dev/null || echo "No logs available"
    fi
}

# Function for full deployment (build + deploy)
full_deployment() {
    if [[ -n "$REMOTE_HOST" ]]; then
        echo -e "${BLUE}🌐 Full remote deployment to: ${YELLOW}$REMOTE_HOST${NC}"
        build_remote
        deploy_container
    else
        echo -e "${BLUE}🏠 Full local deployment${NC}"
        build_local
        deploy_container
    fi
}

# Parse arguments
case ${1} in
    --build)
        build_local
        ;;
    --build-remote)
        shift
        REMOTE_HOST=${1:-$REMOTE_HOST}
        build_remote
        ;;
    --deploy)
        shift
        REMOTE_HOST=${1:-$REMOTE_HOST}
        DRONE_ID=${2:-$DRONE_ID}
        deploy_container
        ;;
    --stop)
        shift
        REMOTE_HOST=${1:-$REMOTE_HOST}
        stop_container
        ;;
    --logs)
        shift
        REMOTE_HOST=${1:-$REMOTE_HOST}
        show_logs
        ;;
    --shell)
        shift
        REMOTE_HOST=${1:-$REMOTE_HOST}
        access_shell
        ;;
    --status)
        shift
        REMOTE_HOST=${1:-$REMOTE_HOST}
        show_status
        ;;
    --help|-h)
        show_usage
        ;;
    *)
        # Default: full deployment
        full_deployment
        ;;
esac