#!/bin/bash

# 查看BEV测试结果

echo "=========================================="
echo "BEV测试结果查看"
echo "=========================================="
echo ""

OUTPUT_DIR="./test_bev_output"

if [ ! -d "$OUTPUT_DIR" ]; then
    echo "错误: 找不到输出目录 $OUTPUT_DIR"
    echo "请先运行 ./bin/test_bev_simple"
    exit 1
fi

echo "生成的图像文件:"
echo ""
ls -lh $OUTPUT_DIR/*.png 2>/dev/null

echo ""
echo "=========================================="
echo "图像说明:"
echo "=========================================="
echo ""
echo "1. bev_front.png - 前相机的BEV视图"
echo "2. bev_left.png - 左相机的BEV视图"
echo "3. bev_back.png - 后相机的BEV视图"
echo "4. bev_right.png - 右相机的BEV视图"
echo "5. before_all_calib.png - 拼接后的环绕视图"
echo ""
echo "请检查 before_all_calib.png 中的交通线:"
echo "  - 是否在不同相机间可见?"
echo "  - 是否大致对齐?"
echo "  - 是否有明显的错位或断裂?"
echo ""

# 尝试使用系统工具打开图像（如果可用）
if command -v eog &> /dev/null; then
    echo "使用 eog 打开图像查看器..."
    eog $OUTPUT_DIR/before_all_calib.png &
elif command -v display &> /dev/null; then
    echo "使用 display 打开图像查看器..."
    display $OUTPUT_DIR/before_all_calib.png &
elif command -v xdg-open &> /dev/null; then
    echo "使用 xdg-open 打开图像..."
    xdg-open $OUTPUT_DIR/before_all_calib.png &
else
    echo "无法自动打开图像查看器。"
    echo "请手动打开: $OUTPUT_DIR/before_all_calib.png"
fi

echo ""
echo "如果交通线完全错位，说明需要调整:"
echo "  1. KG参数 (src/optimizer.cpp:464-474)"
echo "  2. Tail大小 (src/optimizer.cpp:512-518)"
echo "  3. 相机高度 (从外参文件加载，当前正确)"
echo ""
