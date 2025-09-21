#!/bin/bash

# Docker command to build and test MecaBridgeSerialProtocol unit tests
# Run this script from the project root directory
LOG_FILE="tb6612_test_$(date +%Y%m%d_%H%M%S).log"
# Function to log and display
log_and_display() {
    echo "$1" | tee -a "$LOG_FILE"
}
echo "Building and testing MecaBridgeSerialProtocol in Docker container..."
log_and_display "Building and testing MecaBridgeSerialProtocol in Docker container..."

# Build the Docker image if it doesn't exist
echo "Building Docker image..."
docker-compose build

# Run the tests in the container
echo "Running tests in Docker container..."
docker-compose run --rm ros2-dev bash -c "
    echo 'Setting up ROS 2 environment...'
    source /opt/ros/humble/setup.bash || echo 'ROS setup not found, continuing...'
    
    echo 'Building drive_arduino package with tests...'
    colcon build --packages-select drive_arduino --cmake-args -DBUILD_TESTING=ON
    
    if [ \$? -eq 0 ]; then
        echo 'Build successful. Running tests...'
        colcon test --packages-select drive_arduino
        
        echo 'Test results:'
        colcon test-result --all --verbose
    else
        echo 'Build failed.'
        exit 1
    fi
"

echo "Docker test run complete."