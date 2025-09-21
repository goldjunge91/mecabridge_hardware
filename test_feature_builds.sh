#!/bin/bash

# Test script for MecaBridge with disabled features
# Tests different combinations of feature flags to ensure proper compilation and interface behavior

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}Testing MecaBridge with disabled features...${NC}"

# Base directory
BASE_DIR=$(pwd)
BUILD_BASE_DIR="build_feature_tests"

# Test configurations
declare -a CONFIGS=(
    "all_enabled:MECABRIDGE_ENABLE_SERVOS=ON,MECABRIDGE_ENABLE_ESCS=ON,MECABRIDGE_ENABLE_DIAGNOSTICS=ON"
    "servos_disabled:MECABRIDGE_ENABLE_SERVOS=OFF,MECABRIDGE_ENABLE_ESCS=ON,MECABRIDGE_ENABLE_DIAGNOSTICS=ON"
    "escs_disabled:MECABRIDGE_ENABLE_SERVOS=ON,MECABRIDGE_ENABLE_ESCS=OFF,MECABRIDGE_ENABLE_DIAGNOSTICS=ON"
    "servos_escs_disabled:MECABRIDGE_ENABLE_SERVOS=OFF,MECABRIDGE_ENABLE_ESCS=OFF,MECABRIDGE_ENABLE_DIAGNOSTICS=ON"
    "diagnostics_disabled:MECABRIDGE_ENABLE_SERVOS=ON,MECABRIDGE_ENABLE_ESCS=ON,MECABRIDGE_ENABLE_DIAGNOSTICS=OFF"
    "minimal:MECABRIDGE_ENABLE_SERVOS=OFF,MECABRIDGE_ENABLE_ESCS=OFF,MECABRIDGE_ENABLE_DIAGNOSTICS=OFF"
)

# Function to build and test a configuration
test_configuration() {
    local config_name=$1
    local cmake_args=$2
    
    echo -e "${YELLOW}Testing configuration: $config_name${NC}"
    
    # Create build directory for this configuration
    BUILD_DIR="${BUILD_BASE_DIR}/${config_name}"
    mkdir -p "$BUILD_DIR"
    cd "$BUILD_DIR"
    
    # Configure with CMake
    echo "Configuring with: $cmake_args"
    IFS=',' read -ra CMAKE_OPTIONS <<< "$cmake_args"
    CMAKE_CMD="cmake $BASE_DIR"
    for option in "${CMAKE_OPTIONS[@]}"; do
        CMAKE_CMD="$CMAKE_CMD -D$option"
    done
    
    eval $CMAKE_CMD
    
    # Build the package
    echo "Building..."
    make -j"$(nproc)" mecabridge_hardware
    
    # Run feature toggle tests
    echo "Running feature toggle tests..."
    make test_feature_toggles
    if ./test/test_feature_toggles; then
        echo -e "${GREEN}✓ Feature toggle tests passed for $config_name${NC}"
    else
        echo -e "${RED}✗ Feature toggle tests failed for $config_name${NC}"
        cd "$BASE_DIR"
        return 1
    fi
    
    # Try to build other tests to ensure they still work
    echo "Building other tests..."
    make test_config test_crc16 test_frame_roundtrip || {
        echo -e "${RED}✗ Other tests failed to build for $config_name${NC}"
        cd "$BASE_DIR"
        return 1
    }
    
    echo -e "${GREEN}✓ Configuration $config_name passed all tests${NC}"
    cd "$BASE_DIR"
    return 0
}

# Clean up any previous test builds
if [ -d "$BUILD_BASE_DIR" ]; then
    echo "Cleaning up previous test builds..."
    rm -rf "$BUILD_BASE_DIR"
fi

# Test each configuration
FAILED_CONFIGS=()

for config in "${CONFIGS[@]}"; do
    IFS=':' read -ra PARTS <<< "$config"
    CONFIG_NAME="${PARTS[0]}"
    CMAKE_ARGS="${PARTS[1]}"
    
    if ! test_configuration "$CONFIG_NAME" "$CMAKE_ARGS"; then
        FAILED_CONFIGS+=("$CONFIG_NAME")
    fi
done

# Report results
echo
echo "=== Test Results ==="
if [ ${#FAILED_CONFIGS[@]} -eq 0 ]; then
    echo -e "${GREEN}✓ All feature configurations passed!${NC}"
    echo "Tested configurations:"
    for config in "${CONFIGS[@]}"; do
        IFS=':' read -ra PARTS <<< "$config"
        echo "  - ${PARTS[0]}"
    done
else
    echo -e "${RED}✗ The following configurations failed:${NC}"
    for failed in "${FAILED_CONFIGS[@]}"; do
        echo "  - $failed"
    done
    exit 1
fi

echo
echo -e "${GREEN}Feature toggle testing completed successfully!${NC}"