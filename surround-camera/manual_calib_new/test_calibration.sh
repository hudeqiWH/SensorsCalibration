#!/bin/bash

# Test script for new manual_calib

echo "=========================================="
echo "Testing AVM Manual Calibration (New Format)"
echo "=========================================="

BIN_DIR="./bin"
TEST_DATA_DIR="./test_data"

# Check executable
if [ ! -f "$BIN_DIR/run_avm_new" ]; then
    echo "Error: run_avm_new not found in $BIN_DIR"
    exit 1
fi

echo ""
echo "Test 1: Help message"
echo "--------------------"
$BIN_DIR/run_avm_new

echo ""
echo "=========================================="
echo "New manual_calib features:"
echo "=========================================="
echo "1. Supports new Ocam parameter format (intrinsic_param with cam2world/world2cam polynomials)"
echo "2. Supports new extrinsic format (rotation vector + translation)"
echo "3. CPAC-based BEV projection using body coordinate system"
echo "4. Supports camera models: 0=fisheye, 1=ocam, 2=pinhole"
echo ""
echo "Usage examples:"
echo "  Ocam model:"
echo "    ./run_avm_new ./imgs ./ocam_params ./extrinsics.json 1"
echo ""
echo "  Pinhole model:"
echo "    ./run_avm_new ./imgs ./intrinsics ./extrinsics.json 2"
echo ""
echo "Expected file structure:"
echo "  ./imgs/"
echo "    ├── Front.png"
echo "    ├── Left.png"
echo "    ├── Back.png"
echo "    └── Right.png"
echo ""
echo "  ./ocam_params/ (for Ocam model)"
echo "    ├── park_front.json  (with intrinsic_param.cam2world/world2cam)"
echo "    ├── park_left.json"
echo "    ├── park_back.json"
echo "    └── park_right.json"
echo ""
echo "  ./extrinsics.json (new format)"
echo "    {"
echo "      \"extrinsic_param\": {"
echo "        \"park_front\": {\"rotation\": [...], \"translation\": [...]},"
echo "        \"park_left\": {...},"
echo "        \"park_back\": {...},"
echo "        \"park_right\": {...}"
echo "      }"
echo "    }"
echo ""
