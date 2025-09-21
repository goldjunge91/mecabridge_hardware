#!/bin/bash

# Development helper script

set -e

echo "🐳 Starting ROS 2 development environment..."

# Build and start the container
docker-compose up --build -d

# Enter the container
echo "🔧 Entering development container..."
docker-compose exec ros2-dev bash

echo "👋 Exited development environment"