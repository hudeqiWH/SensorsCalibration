#!/bin/bash

# 诊断脚本：快速检查BEV测试结果

echo "=========================================="
echo "BEV测试诊断工具"
echo "=========================================="
echo ""

cd /home/nio/deqi/thirdparty/SensorsCalibration/surround-camera/auto_calib_fisheye

# 检查测试输出是否存在
if [ ! -d "test_bev_output" ]; then
    echo "❌ 错误: 找不到测试输出目录"
    echo "   请先运行: ./bin/test_bev_simple"
    exit 1
fi

echo "✓ 找到测试输出:"
echo ""

# 列出所有输出文件
for file in test_bev_output/*.png; do
    if [ -f "$file" ]; then
        size=$(ls -lh "$file" | awk '{print $5}')
        echo "  - $(basename $file): $size"
    fi
done

echo ""
echo "=========================================="
echo "分析结果"
echo "=========================================="
echo ""

# 检查文件大小
main_file="test_bev_output/before_all_calib.png"
main_size=$(stat -c%s "$main_file" 2>/dev/null || stat -f%z "$main_file" 2>/dev/null)

if [ "$main_size" -lt 50000 ]; then
    echo "⚠️  警告: $main_file 文件很小 ($main_size 字节)"
    echo "    这可能意味着:"
    echo "    1. BEV图像大部分是黑色/单色"
    echo "    2. 交通线没有正确投影"
    echo "    3. 投影参数可能有误"
    echo ""
    echo "建议检查:"
    echo "    - Ocam投影函数的数学实现"
    echo "    - 图像旋转方向"
    echo "    - KG参数（地面分辨率）"
else
    echo "✓ 文件大小正常 ($main_size 字节)"
    echo "  请手动查看图像确认交通线是否可见"
fi

echo ""
echo "=========================================="
echo "如何查看图像"
echo "=========================================="
echo ""
echo "方法1: 使用图像查看器"
echo "  eog ./test_bev_output/before_all_calib.png"
echo ""
echo "方法2: 复制到本地机器查看"
echo "  scp /home/nio/deqi/thirdparty/SensorsCalibration/surround-camera/auto_calib_fisheye/test_bev_output/before_all_calib.png ~/Desktop/"
echo ""
echo "方法3: 使用xdg-open"
echo "  xdg-open ./test_bev_output/before_all_calib.png"
echo ""
echo "=========================================="
echo "检查要点"
echo "=========================================="
echo ""
echo "查看 ./test_bev_output/before_all_calib.png:"
echo "  1. 是否能看到任何图像内容？（不只是黑色）"
echo "  2. 交通线（如道路标线）是否可见？"
echo "  3. 四个相机的图像是否大致对齐？"
echo ""
echo "如果完全看不到内容，问题在投影函数"
echo "如果能看到内容但不对齐，问题在外参或KG"
echo ""
