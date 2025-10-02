#!/bin/bash

# Local Docker Build and Transfer Script
# This script builds the image locally and transfers it to the remote device

set -e

# Configuration
REMOTE_HOST="10.42.0.64"
REMOTE_USER="compulab"
IMAGE_NAME="1nrobotics/shiviz_px4_nav"
TAG="local"
DRONE_ID=${DRONE_ID:-1}

echo "🚁 Local Docker Build and Transfer"
echo "Target: $REMOTE_USER@$REMOTE_HOST"
echo "Drone ID: $DRONE_ID"

# Step 1: Build image locally
echo ""
echo "🔨 Step 1: Building Docker image locally..."
echo "Building for ARM64 architecture..."

# Build for ARM64 (assuming your local machine has buildx support)
if docker buildx version >/dev/null 2>&1; then
    docker buildx build --platform linux/arm64 -t $IMAGE_NAME:$TAG .
else
    echo "⚠️  buildx not available, building for current platform..."
    docker build -t $IMAGE_NAME:$TAG .
fi

# Step 2: Save image to tar file
echo ""
echo "💾 Step 2: Saving Docker image..."
docker save $IMAGE_NAME:$TAG > shiviz_image.tar
echo "Image saved as shiviz_image.tar ($(du -h shiviz_image.tar | cut -f1))"

# Step 3: Transfer image to remote
echo ""
echo "📤 Step 3: Transferring image to remote device..."
echo "This will require password authentication..."
scp shiviz_image.tar $REMOTE_USER@$REMOTE_HOST:~/

# Step 4: Load image on remote device
echo ""
echo "📥 Step 4: Loading image on remote device..."
echo "This will require sudo password on the remote device."
ssh -t $REMOTE_USER@$REMOTE_HOST "sudo docker load < ~/shiviz_image.tar && rm ~/shiviz_image.tar"

# Step 5: Run container
echo ""
echo "🚀 Step 5: Starting container..."
DOCKER_CMD="sudo docker run -d --name shiviz_nav_$DRONE_ID --restart unless-stopped -e DRONE_ID=$DRONE_ID $IMAGE_NAME:$TAG"
ssh -t $REMOTE_USER@$REMOTE_HOST "$DOCKER_CMD"

# Clean up local tar file
rm shiviz_image.tar

echo ""
echo "✅ Deployment complete!"
echo "Container 'shiviz_nav_$DRONE_ID' is running on $REMOTE_HOST"
echo ""
echo "To check status: ssh $REMOTE_USER@$REMOTE_HOST 'sudo docker ps'"
echo "To view logs: ssh $REMOTE_USER@$REMOTE_HOST 'sudo docker logs shiviz_nav_$DRONE_ID'"
echo "To stop: ssh $REMOTE_USER@$REMOTE_HOST 'sudo docker stop shiviz_nav_$DRONE_ID'"