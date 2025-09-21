#!/bin/bash

# Test runner script for MecaBridgeSerialProtocol unit tests
# This script can be used to run the tests manually in Docker environment

echo "Building and running MecaBridgeSerialProtocol unit tests..."

# Try to find and source ROS 2 environment
if [ -f "/opt/ros/humble/setup.bash" ]; then
    echo "Sourcing ROS 2 from /opt/ros/humble/setup.bash"
    source /opt/ros/humble/setup.bash
elif [ -f "/ros_entrypoint.sh" ]; then
    echo "Using ROS entrypoint script"
    source /ros_entrypoint.sh
else
    echo "Warning: Could not find ROS 2 setup script. Proceeding anyway..."
fi

# Build the package with tests
echo "Building drive_arduino package with tests..."
colcon build --packages-select drive_arduino --cmake-args -DBUILD_TESTING=ON

if [ $? -eq 0 ]; then
    echo "Build successful. Running tests..."
    
    # Run the tests
    colcon test --packages-select drive_arduino
    
    if [ $? -eq 0 ]; then
        echo "Tests completed. Showing results..."
        # Show test results
        colcon test-result --all --verbose
    else
        echo "Tests failed. Showing results..."
        colcon test-result --all --verbose
    fi
else
    echo "Build failed. Cannot run tests."
    exit 1
fi