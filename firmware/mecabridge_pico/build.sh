#!/bin/bash

# MecaBridge Pico Firmware Build Script
# Builds the UF2 file for the Raspberry Pi Pico

set -e  # Exit on error

# Configuration
BUILD_DIR="build"
TARGET_SIZE_KB=256  # Target maximum size in KB
PROJECT_NAME="MecaBridge"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}Building MecaBridge Pico Firmware...${NC}"

# Check if PICO_SDK_PATH is set
if [ -z "$PICO_SDK_PATH" ]; then
    echo -e "${RED}Error: PICO_SDK_PATH environment variable not set${NC}"
    echo "Please set PICO_SDK_PATH to your Pico SDK installation directory"
    exit 1
fi

# Create build directory if it doesn't exist
if [ ! -d "$BUILD_DIR" ]; then
    echo "Creating build directory..."
    mkdir -p "$BUILD_DIR"
fi

# Navigate to build directory
cd "$BUILD_DIR"

# Configure with CMake
echo "Configuring CMake..."
cmake .. -DCMAKE_BUILD_TYPE=Release

# Build the project
echo "Building project..."
make -j"$(nproc)"

# Check if build was successful
if [ ! -f "${PROJECT_NAME}.uf2" ]; then
    echo -e "${RED}Error: Build failed - ${PROJECT_NAME}.uf2 not found${NC}"
    exit 1
fi

# Check file size
UF2_SIZE_BYTES=$(stat -c%s "${PROJECT_NAME}.uf2")
UF2_SIZE_KB=$((UF2_SIZE_BYTES / 1024))

echo -e "${GREEN}Build successful!${NC}"
echo "Generated files:"
echo "  - ${PROJECT_NAME}.uf2 (${UF2_SIZE_KB} KB)"
echo "  - ${PROJECT_NAME}.elf"
echo "  - ${PROJECT_NAME}.bin"

# Check size constraint
if [ $UF2_SIZE_KB -gt $TARGET_SIZE_KB ]; then
    echo -e "${YELLOW}Warning: UF2 size (${UF2_SIZE_KB} KB) exceeds target (${TARGET_SIZE_KB} KB)${NC}"
    echo "Consider optimizing the firmware to reduce size."
else
    echo -e "${GREEN}Size check passed: ${UF2_SIZE_KB} KB <= ${TARGET_SIZE_KB} KB${NC}"
fi

# Show memory usage if available
if [ -f "${PROJECT_NAME}.elf" ]; then
    echo
    echo "Memory usage:"
    arm-none-eabi-size "${PROJECT_NAME}.elf" || echo "arm-none-eabi-size not available"
fi

echo
echo -e "${GREEN}To flash the firmware:${NC}"
echo "1. Hold BOOTSEL button on Pico and connect to USB"
echo "2. Copy ${PROJECT_NAME}.uf2 to the RPI-RP2 drive"
echo "   OR"
echo "3. Run: make flash (if picotool is installed)"

cd ..