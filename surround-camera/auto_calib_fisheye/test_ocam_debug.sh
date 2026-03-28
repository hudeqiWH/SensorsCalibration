#!/bin/bash

# Ocam标定调试脚本 - 包含完整的验证和测试

set -e

echo "=========================================="
echo "Ocam标定调试测试"
echo "=========================================="
echo ""

# 设置路径
WORK_DIR="/home/nio/deqi/thirdparty/SensorsCalibration/surround-camera/auto_calib_fisheye"
IMG_DIR="$WORK_DIR/2_dog/imgs"
PARAM_DIR="$WORK_DIR/2_dog/param"
EXTRINSICS_FILE="/home/nio/deqi/r_calib/data/trans/camera_extrinsics_ikalibr.json5"
OUTPUT_DIR="$WORK_DIR/test_output_debug"

# 创建输出目录
mkdir -p $OUTPUT_DIR

echo "工作目录: $WORK_DIR"
echo "图像目录: $IMG_DIR"
echo "参数目录: $PARAM_DIR"
echo "外参文件: $EXTRINSICS_FILE"
echo "输出目录: $OUTPUT_DIR"
echo ""

# 检查输入文件
echo "检查输入文件..."
missing_files=0

# 检查图像文件
for cam in front left back right; do
  img_file="$IMG_DIR/Park${cam^}*.png"
  if ls $img_file 1> /dev/null 2>&1; then
    echo "✓ 找到 $cam 相机图像"
  else
    echo "✗ 缺少 $cam 相机图像"
    missing_files=$((missing_files + 1))
  fi
done

# 检查参数文件
for cam in front left back right; do
  param_file="$PARAM_DIR/park_${cam}.json"
  if [ -f "$param_file" ]; then
    echo "✓ 找到 $cam 相机参数: $param_file"
  else
    echo "✗ 缺少 $cam 相机参数: $param_file"
    missing_files=$((missing_files + 1))
  fi
done

# 检查外参文件
if [ -f "$EXTRINSICS_FILE" ]; then
  echo "✓ 找到外参文件: $EXTRINSICS_FILE"
else
  echo "✗ 缺少外参文件: $EXTRINSICS_FILE"
  missing_files=$((missing_files + 1))
fi

echo ""

if [ $missing_files -gt 0 ]; then
  echo "错误: 缺少 $missing_files 个文件！"
  exit 1
fi

echo "所有必需文件已找到！"
echo ""

# 编译项目
echo "=========================================="
echo "编译项目..."
echo "=========================================="
cd $WORK_DIR
if [ -d "build" ]; then
  rm -rf build
fi
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Debug
make -j$(nproc)
echo "编译完成！"
echo ""

# 检查可执行文件
if [ ! -f "$WORK_DIR/bin/run_AVM_Calibration" ]; then
  echo "错误: 编译后未找到可执行文件 $WORK_DIR/bin/run_AVM_Calibration"
  exit 1
fi
echo "✓ 可执行文件已生成: $WORK_DIR/bin/run_AVM_Calibration"
echo ""

# 运行标定
echo "=========================================="
echo "运行Ocam标定..."
echo "=========================================="
echo ""

# 找到第一张图像
FRONT_IMG=$(ls $IMG_DIR/ParkFront*.png | head -n 1)
LEFT_IMG=$(ls $IMG_DIR/ParkLeft*.png | head -n 1)
BACK_IMG=$(ls $IMG_DIR/ParkBack*.png | head -n 1)
RIGHT_IMG=$(ls $IMG_DIR/ParkRight*.png | head -n 1)

if [ -z "$FRONT_IMG" ] || [ -z "$LEFT_IMG" ] || [ -z "$BACK_IMG" ] || [ -z "$RIGHT_IMG" ]; then
  echo "错误: 无法找到所有相机的图像文件"
  exit 1
fi

echo "使用图像:"
echo "  前: $FRONT_IMG"
echo "  左: $LEFT_IMG"
echo "  后: $BACK_IMG"
echo "  右: $RIGHT_IMG"
echo ""

# 运行标定
cd $WORK_DIR
./bin/run_AVM_Calibration \
  --camera-model 1 \
  --front "$FRONT_IMG" \
  --left "$LEFT_IMG" \
  --behind "$BACK_IMG" \
  --right "$RIGHT_IMG" \
  --ocam-front "$PARAM_DIR/park_front.json" \
  --ocam-left "$PARAM_DIR/park_left.json" \
  --ocam-behind "$PARAM_DIR/park_back.json" \
  --ocam-right "$PARAM_DIR/park_right.json" \
  --extrinsics "$EXTRINSICS_FILE" \
  --output-dir "$OUTPUT_DIR/"

echo ""
echo "标定完成！"
echo ""

# 验证输出
echo "=========================================="
echo "验证输出结果..."
echo "=========================================="
echo ""

output_files=(
  "$OUTPUT_DIR/before_all_calib.png"
  "$OUTPUT_DIR/after_all_calib.png"
  "$OUTPUT_DIR/calibration_results.json5"
)

missing_output=0
for file in "${output_files[@]}"; do
  if [ -f "$file" ]; then
    echo "✓ 生成文件: $file"
    
    # 显示文件信息
    if [[ $file == *.png ]]; then
      file_size=$(ls -lh $file | awk '{print $5}')
      echo "  大小: $file_size"
    elif [[ $file == *.json5 ]]; then
      echo "  内容预览:"
      head -20 $file | sed 's/^/    /'
    fi
  else
    echo "✗ 缺少文件: $file"
    missing_output=$((missing_output + 1))
  fi
  echo ""
done

# 检查标定结果质量
echo "=========================================="
echo "检查标定质量..."
echo "=========================================="
echo ""

if [ -f "$OUTPUT_DIR/calibration_results.json5" ]; then
  echo "外参结果:"
  echo ""
  
  # 使用Python解析JSON
  python3 << EOF
import json5
import json

with open('$OUTPUT_DIR/calibration_results.json5', 'r') as f:
    data = json5.load(f)

extrinsics = data.get('extrinsic_param', {})

for cam_name in ['park_front', 'park_left', 'park_back', 'park_right']:
    if cam_name in extrinsics:
        cam = extrinsics[cam_name]
        rotation = cam['rotation']
        translation = cam['translation']
        
        # 计算旋转的模（角度）
        import math
        angle = math.sqrt(rotation[0]**2 + rotation[1]**2 + rotation[2]**2)
        angle_deg = angle * 180 / math.pi
        
        print(f"{cam_name}:")
        print(f"  旋转向量: [{rotation[0]:.6f}, {rotation[1]:.6f}, {rotation[2]:.6f}]")
        print(f"  旋转角度: {angle_deg:.2f}°")
        print(f"  平移: [{translation[0]:.6f}, {translation[1]:.6f}, {translation[2]:.6f}]")
        print(f"  相机高度: {translation[2]*100:.1f} cm")
        print("")
EOF
fi

# 总结
echo "=========================================="
echo "测试完成！"
echo "=========================================="
echo ""

if [ $missing_output -eq 0 ]; then
  echo "✓ 所有测试通过！"
  echo ""
  echo "结果保存在: $OUTPUT_DIR/"
  echo ""
  echo "关键文件:"
  echo "  - before_all_calib.png: 标定前的BEV视图"
  echo "  - after_all_calib.png: 标定后的BEV视图"
  echo "  - calibration_results.json5: 标定结果外参"
  echo ""
  echo "请检查 after_all_calib.png 中的交通线是否对齐！"
  exit 0
else
  echo "✗ 测试失败！缺少 $missing_output 个输出文件"
  exit 1
fi
