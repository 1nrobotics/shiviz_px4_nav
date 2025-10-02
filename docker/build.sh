#!/bin/bash

# Shiviz PX4 Navigation Docker Build Script
# Based on Gestelt framework Docker setup

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Configuration
REGISTRY=${REGISTRY:-"ghcr.io"}  # Can be overridden with environment variable
IMAGE_NAME="${REGISTRY}/1nrobotics/shiviz_px4_nav"
TAG=${1:-"latest"}
PLATFORM=${2:-"linux/amd64"}
BUILD_TYPE=${3:-"Release"}

echo -e "${BLUE}🚁 Shiviz PX4 Navigation Docker Build${NC}"
echo -e "Image: ${GREEN}${IMAGE_NAME}:${TAG}${NC}"
echo -e "Platform: ${YELLOW}${PLATFORM}${NC}"
echo -e "Build Type: ${YELLOW}${BUILD_TYPE}${NC}"
echo ""

# Check if Docker is installed
if ! command -v docker &> /dev/null; then
    echo -e "${RED}❌ Docker is not installed${NC}"
    exit 1
fi

# Check if docker-compose is installed  
if ! command -v docker-compose &> /dev/null; then
    echo -e "${YELLOW}⚠️  docker-compose not found, installing...${NC}"
    sudo apt update && sudo apt install docker-compose -y
fi

# Function to build image
build_image() {
    echo -e "${BLUE}🔨 Building Docker image...${NC}"
    
    if [[ "$PLATFORM" == "linux/arm64" ]]; then
        # ARM64 build for Radxa deployment
        docker buildx build \
            --platform "$PLATFORM" \
            --build-arg CMAKE_BUILD_TYPE="$BUILD_TYPE" \
            -t "${IMAGE_NAME}:${TAG}" \
            -t "${IMAGE_NAME}:arm64" \
            --push \
            .
    else
        # Standard AMD64 build
        docker build \
            --build-arg CMAKE_BUILD_TYPE="$BUILD_TYPE" \
            -t "${IMAGE_NAME}:${TAG}" \
            .
    fi
    
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✅ Build completed successfully${NC}"
    else
        echo -e "${RED}❌ Build failed${NC}"
        exit 1
    fi
}

# Function to test image
test_image() {
    echo -e "${BLUE}🧪 Testing Docker image...${NC}"
    
    # Test basic functionality
    docker run --rm "${IMAGE_NAME}:${TAG}" bash -c "
        source /catkin_ws/devel/setup.bash && 
        rospack find shiviz_px4_nav &&
        echo 'ROS package found successfully'
    "
    
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✅ Image test passed${NC}"
    else
        echo -e "${RED}❌ Image test failed${NC}"
        exit 1
    fi
}

# Function to push image
push_image() {
    if [[ "$TAG" != "latest" ]] || [[ "$PLATFORM" == "linux/arm64" ]]; then
        echo -e "${BLUE}🚀 Pushing image to registry...${NC}"
        docker push "${IMAGE_NAME}:${TAG}"
        
        if [[ "$PLATFORM" == "linux/arm64" ]]; then
            docker push "${IMAGE_NAME}:arm64"
        fi
        
        echo -e "${GREEN}✅ Image pushed successfully${NC}"
    else
        echo -e "${YELLOW}⚠️  Skipping push for local latest build${NC}"
    fi
}

# Function to clean old images
clean_images() {
    echo -e "${BLUE}🧹 Cleaning up old images...${NC}"
    
    # Remove dangling images
    docker image prune -f
    
    # Remove old versions (keep latest 3)
    docker images "${IMAGE_NAME}" --format "table {{.Tag}}\t{{.ID}}" | \
    tail -n +2 | head -n -3 | awk '{print $2}' | \
    xargs -r docker rmi
    
    echo -e "${GREEN}✅ Cleanup completed${NC}"
}

# Function to show usage
show_usage() {
    echo "Usage: $0 [TAG] [PLATFORM] [BUILD_TYPE]"
    echo ""
    echo "Arguments:"
    echo "  TAG         Docker tag (default: latest)"
    echo "  PLATFORM    Target platform (default: linux/amd64)"
    echo "  BUILD_TYPE  CMake build type (default: Release)"
    echo ""
    echo "Examples:"
    echo "  $0                              # Build latest for AMD64"
    echo "  $0 v1.0.0                      # Build v1.0.0 for AMD64"
    echo "  $0 latest linux/arm64          # Build for ARM64 (Radxa)"
    echo "  $0 debug linux/amd64 Debug     # Debug build for AMD64"
    echo ""
    echo "Profiles:"
    echo "  --simulation    Build with simulation support"
    echo "  --development   Build with development tools"
    echo "  --clean         Clean old images before build"
    echo "  --no-test       Skip image testing"
    echo "  --push          Push to registry after build"
}

# Parse additional flags
SIMULATION=false
DEVELOPMENT=false  
CLEAN=false
NO_TEST=false
PUSH=false

while [[ $# -gt 0 ]]; do
    case $1 in
        --simulation)
            SIMULATION=true
            shift
            ;;
        --development)
            DEVELOPMENT=true
            shift
            ;;
        --clean)
            CLEAN=true
            shift
            ;;
        --no-test)
            NO_TEST=true
            shift
            ;;
        --push)
            PUSH=true
            shift
            ;;
        --help|-h)
            show_usage
            exit 0
            ;;
        *)
            break
            ;;
    esac
done

# Main execution
main() {
    echo -e "${BLUE}Starting build process...${NC}"
    
    # Clean if requested
    if [[ "$CLEAN" == "true" ]]; then
        clean_images
    fi
    
    # Build the image
    build_image
    
    # Test unless skipped
    if [[ "$NO_TEST" != "true" ]]; then
        test_image
    fi
    
    # Push if requested
    if [[ "$PUSH" == "true" ]]; then
        push_image
    fi
    
    echo ""
    echo -e "${GREEN}🎉 Build process completed successfully!${NC}"
    echo -e "Image: ${GREEN}${IMAGE_NAME}:${TAG}${NC}"
    
    # Show usage examples
    echo ""
    echo -e "${BLUE}Quick start commands:${NC}"
    echo "  docker run -it --rm --network host ${IMAGE_NAME}:${TAG}"
    echo "  docker-compose up"
    echo "  docker-compose --profile development up shiviz_dev"
}

# Run main function
main "$@"