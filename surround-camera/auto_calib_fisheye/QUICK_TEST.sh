#!/bin/bash

# 快速测试：查看BEV图像

cd /home/nio/deqi/thirdparty/SensorsCalibration/surround-camera/auto_calib_fisheye

if [ ! -f "./test_bev_output/before_all_calib.png" ]; then
    echo "错误: 找不到测试输出图像"
    echo "请先运行: ./bin/test_bev_simple"
    exit 1
fi

echo "=========================================="
echo "查看BEV测试结果"
echo "=========================================="
echo ""
echo "文件: ./test_bev_output/before_all_calib.png"
echo "大小: $(ls -lh ./test_bev_output/before_all_calib.png | awk '{print $5}')"
echo ""
echo "请手动打开此文件并检查："
echo "  1. 是否能看到任何图像内容？"
echo "  2. 交通线是否可见？"
echo "  3. 四个相机的图像是否对齐？"
echo ""
echo "如果完全看不到内容，说明投影函数有问题"
echo "如果能看到内容但交通线不对齐，说明外参或KG有问题"
echo ""

# 尝试打开图像查看器
echo "尝试打开图像查看器..."
if command -v eog &> /dev/null; then
    eog ./test_bev_output/before_all_calib.png &
elif command -v display &> /dev/null; then
    display ./test_bev_output/before_all_calib.png &
elif command -v xdg-open &> /dev/null; then
    xdg-open ./test_bev_output/before_all_calib.png &
else
    echo "无法自动打开图像查看器"
    echo "请手动打开: ./test_bev_output/before_all_calib.png"
fi

echo ""
echo "=========================================="
echo "检查单个相机BEV图像:"
echo "=========================================="
echo ""
ls -lh ./test_bev_output/bev_*.png
