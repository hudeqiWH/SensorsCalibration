#!/bin/bash

# 测试仅生成BEV图像，不进行纹理提取和优化

cd /home/nio/deqi/thirdparty/SensorsCalibration/surround-camera/auto_calib_fisheye

# 编译
cd build
make -j$(nproc)
cd ..

# 运行简化测试 - 只生成before_all_calib.png
echo "=========================================="
echo "测试BEV投影..."
echo "=========================================="

./bin/run_AVM_Calibration \
  --camera-model 1 \
  --front "./2_dog/imgs/ParkFront-1774583477301602000.png" \
  --left "./2_dog/imgs/ParkLeft-1774583477034621000.png" \
  --behind "./2_dog/imgs/ParkBack-1774583477301602000.png" \
  --right "./2_dog/imgs/ParkRight-1774583477101370000.png" \
  --ocam-front "./2_dog/param/park_front.json" \
  --ocam-left "./2_dog/param/park_left.json" \
  --ocam-behind "./2_dog/param/park_back.json" \
  --ocam-right "./2_dog/param/park_right.json" \
  --extrinsics "/home/nio/deqi/r_calib/data/trans/camera_extrinsics_ikalibr.json5" \
  --output-dir "./test_bev_only/" 2>&1 | head -80

echo ""
echo "检查生成的图像..."
if [ -f "./test_bev_only/before_all_calib.png" ]; then
  echo "✓ BEV图像已生成: ./test_bev_only/before_all_calib.png"
  ls -lh ./test_bev_only/*.png
else
  echo "✗ BEV图像未生成"
fi
