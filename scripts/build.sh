#!/bin/bash

# Build script for ROS 2 workspace in Docker

set -e

echo "🚀 Starting ROS 2 build process..."

# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Clean previous build if requested
if [ "$1" = "--clean" ]; then
    echo "🧹 Cleaning previous build artifacts..."
    rm -rf build/ install/ log/
    # Also clean drive_arduino build if it exists
    if [ -d "drive_arduino/build" ]; then
        rm -rf drive_arduino/build/ drive_arduino/install/ drive_arduino/log/
    fi
fi

# Set up proper workspace structure if needed
if [ -d "drive_arduino" ] && [ ! -d "src/drive_arduino" ]; then
    echo "📁 Setting up proper ROS 2 workspace structure..."
    mkdir -p src
    mv drive_arduino src/
fi

# Build the workspace
echo "🔨 Building workspace..."
if [ -d "src" ]; then
    # Standard ROS 2 workspace structure
    colcon build --symlink-install
else
    # Fallback: build from current directory
    colcon build --symlink-install
fi

# Source the workspace
if [ -f install/setup.bash ]; then
    source install/setup.bash
    echo "✅ Build completed successfully!"
    echo "📦 Available packages:"
    ros2 pkg list | grep -E "(robot|mecabridge)" || echo "No custom packages found"
else
    echo "❌ Build failed - install/setup.bash not found"
    exit 1
fi