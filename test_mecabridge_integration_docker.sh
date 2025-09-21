#!/bin/bash

# Docker command to build and run TB6612 integration tests
# Run this script from the project root directory

LOG_FILE="tb6612_integration_test_$(date +%Y%m%d_%H%M%S).log"

# Function to log and display
log_and_display() {
    echo "$1" | tee -a "$LOG_FILE"
}

log_and_display "========================================="
log_and_display "TB6612 Hardware Interface Integration Tests"
log_and_display "========================================="

# Build the Docker image if it doesn't exist
echo "Building Docker image..."
docker-compose build

# Run the integration tests in the container
echo "Running integration tests in Docker container..."
docker-compose run --rm ros2-dev bash -c "
    echo 'Setting up ROS 2 environment...'
    source /opt/ros/humble/setup.bash
    
    echo 'Building drive_arduino package with tests enabled...'
    colcon build --packages-select drive_arduino --cmake-args -DBUILD_TESTING=ON
    
    if [ \$? -eq 0 ]; then
        echo 'Build successful. Sourcing workspace...'
        source install/setup.bash
        
        echo '========================================='
        echo 'Running Integration Tests'
        echo '========================================='
        
        # Run C++ integration tests
        echo 'Running C++ integration tests...'
        colcon test --packages-select drive_arduino --ctest-args -R test_mecabridge_integration
        
        # Run Python launch file tests
        echo 'Running Python launch file tests...'
        python3 src/drive_arduino/test/test_launch_functionality.py
        
        # Run example usage tests (static tests only)
        echo 'Running example usage tests...'
        python3 src/drive_arduino/test/example_usage.py
        
        echo '========================================='
        echo 'Test Results Summary'
        echo '========================================='
        colcon test-result --all --verbose
        
        echo '========================================='
        echo 'Integration Test Complete'
        echo '========================================='
        
    else
        echo 'Build failed.'
        exit 1
    fi
" 2>&1 | tee -a "$LOG_FILE"

echo "Docker integration test run complete. Log saved to: $LOG_FILE"