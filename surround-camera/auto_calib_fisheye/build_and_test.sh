#!/bin/bash
set -e

echo "=========================================="
echo "Building auto_calib_fisheye with CPAC fixes"
echo "=========================================="

cd /home/nio/deqi/thirdparty/SensorsCalibration/surround-camera/auto_calib_fisheye

# Create build directory
mkdir -p build
cd build

# Configure and build
echo "Configuring with CMake..."
cmake .. -DCMAKE_BUILD_TYPE=Release

echo "Building..."
make -j$(nproc)

echo ""
echo "=========================================="
echo "Build successful! Running BEV test..."
echo "=========================================="
echo ""

# Run the test
cd ..
./build/test_bev_simple

echo ""
echo "=========================================="
echo "Test complete!"
echo "=========================================="
