#!/bin/bash

# Test runner script for MecaBridgeHardwareInterface unit tests
# This script can be used to run the hardware interface tests manually

echo "Building and running MecaBridgeHardwareInterface unit tests..."

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
    echo "Build successful. Running hardware interface tests..."
    
    # Run only the hardware interface tests
    colcon test --packages-select drive_arduino --ctest-args -R test_mecabridge_hardware_interface
    
    if [ $? -eq 0 ]; then
        echo "Hardware interface tests completed. Showing results..."
        # Show test results
        colcon test-result --all --verbose
    else
        echo "Hardware interface tests failed. Showing results..."
        colcon test-result --all --verbose
    fi
else
    echo "Build failed. Cannot run tests."
    exit 1
fi