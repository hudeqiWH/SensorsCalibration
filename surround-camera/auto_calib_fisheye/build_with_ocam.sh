#!/bin/bash
# Build script for auto_calib_fisheye with Ocam support

set -e

echo "=========================================="
echo "Building auto_calib_fisheye with Ocam support"
echo "=========================================="

# Check for required dependencies
echo ""
echo "Checking dependencies..."

# Check for OpenCV
if ! pkg-config --exists opencv4; then
    echo "Error: OpenCV4 not found. Please install libopencv-dev"
    exit 1
fi
echo "✓ OpenCV4 found"

# Check for Eigen3
if ! pkg-config --exists eigen3; then
    echo "Error: Eigen3 not found. Please install libeigen3-dev"
    exit 1
fi
echo "✓ Eigen3 found"

# Check for jsoncpp
if ! pkg-config --exists jsoncpp; then
    echo "Error: jsoncpp not found. Please install libjsoncpp-dev"
    echo "  sudo apt-get install libjsoncpp-dev"
    exit 1
fi
echo "✓ jsoncpp found"

# Create build directory
mkdir -p build
cd build

# Clean previous build (optional)
if [ "$1" == "clean" ]; then
    echo "Cleaning previous build..."
    rm -rf *
fi

# Configure with cmake
echo ""
echo "Configuring with CMake..."
echo "JSONCPP include dirs: $(pkg-config --cflags jsoncpp)"
echo "JSONCPP libraries: $(pkg-config --libs jsoncpp)"

cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_FLAGS="-std=c++17 -pthread"

# Build
echo ""
echo "Building..."
make -j$(nproc)

echo ""
echo "=========================================="
echo "Build completed successfully!"
echo "=========================================="
echo ""
echo "The executable is at: ./bin/run_AVM_Calibration"
echo ""
echo "To run with Ocam cameras:"
echo "  ./bin/run_AVM_Calibration --camera-model 1 \\"
echo "    --ocam-front calib_front.json \\"
echo "    --ocam-left calib_left.json \\"
echo "    --ocam-behind calib_behind.json \\"
echo "    --ocam-right calib_right.json"
echo ""
echo "See OCAM_USAGE.md for detailed documentation."
