# Ocam标定修复后执行指南

## 修复总结

已系统修复Ocam标定算法中的关键问题：

### 主要修复
1. ✅ **修复Ocam投影函数** - 使用实际3D坐标而非固定z=-1
2. ✅ **修复3D坐标传递** - 使用CV_64FC3类型传递完整坐标
3. ✅ **修复坐标变换方向** - 正确转换camera-to-body到body-to-camera
4. ✅ **修复相机高度加载** - 从外参文件自动提取高度信息
5. ✅ **扩大搜索范围** - 旋转±15°，平移±0.05m

### 验证项目
- ✅ 多项式求值 - 正确（Horner方法）
- ✅ 仿射变换 - 正确
- ✅ 编译通过 - 无错误

## 快速测试

```bash
cd /home/nio/deqi/thirdparty/SensorsCalibration/surround-camera/auto_calib_fisheye

# 运行完整测试
./test_ocam_debug.sh
```

测试脚本将自动：
- 检查所有输入文件
- 编译项目
- 运行标定
- 验证输出
- 显示结果摘要

## 手动运行

如果测试脚本失败，可以手动运行：

```bash
# 1. 编译
cd /home/nio/deqi/thirdparty/SensorsCalibration/surround-camera/auto_calib_fisheye
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Debug
make -j$(nproc)

# 2. 运行标定
cd /home/nio/deqi/thirdparty/SensorsCalibration/surround-camera/auto_calib_fisheye

# 找到测试图像
IMG_DIR="./2_dog/imgs"
FRONT_IMG=$(ls $IMG_DIR/ParkFront*.png | head -n 1)
LEFT_IMG=$(ls $IMG_DIR/ParkLeft*.png | head -n 1)
BACK_IMG=$(ls $IMG_DIR/ParkBack*.png | head -n 1)
RIGHT_IMG=$(ls $IMG_DIR/ParkRight*.png | head -n 1)

# 运行标定
./bin/run_AVM_Calibration \
  --camera-model 1 \
  --front "$FRONT_IMG" \
  --left "$LEFT_IMG" \
  --behind "$BACK_IMG" \
  --right "$RIGHT_IMG" \
  --ocam-front "./2_dog/param/park_front.json" \
  --ocam-left "./2_dog/param/park_left.json" \
  --ocam-behind "./2_dog/param/park_back.json" \
  --ocam-right "./2_dog/param/park_right.json" \
  --extrinsics "/home/nio/deqi/r_calib/data/trans/camera_extrinsics_ikalibr.json5" \
  --output-dir "./test_output_debug/"
```

## 预期结果

修复后应看到：

### Loss值改善
- 修复前：~8,000,000（8M）
- 修复后：预计<2,000,000（2M）或更低

### 图像质量
- **before_all_calib.png**：标定前，交通线错位
- **after_all_calib.png**：标定后，交通线应对齐

检查`after_all_calib.png`：
- ✅ 前、左、后、右相机的交通线连续
- ✅ 交叉区域的线条对齐
- ✅ 没有明显的断裂或错位

### 外参文件
输出`calibration_results.json5`包含优化后的外参，格式与iKalibr兼容。

## 调试信息

如果结果仍不理想，检查：

### 1. 查看详细输出
```bash
# 重定向输出到日志
./bin/run_AVM_Calibration ... > debug.log 2>&1

# 查看Loss值变化
grep "luminorsity loss" debug.log
```

### 2. 验证中间结果

每个相机标定后会生成：
- `after_{cam}_calib1.png`：第一次粗调后
- `after_{cam}_calib2.png`：第二次精调后
- `after_{cam}_calib3.png`：第三次精调后

### 3. 检查纹理提取

输出目录中的`texture_*.png`文件显示提取的纹理区域：
- `texture_fl.png`：Front-Left公共区域
- `texture_fr.png`：Front-Right公共区域
- `texture_bl.png`：Back-Left公共区域
- `texture_br.png`：Back-Right公共区域

## 关键文件

- **源代码**：
  - `src/optimizer.cpp` - 核心投影和优化逻辑
  - `src/calibration.cpp` - 主流程和搜索参数
  - `include/optimizer.h` - 头文件

- **测试数据**：
  - `2_dog/imgs/Park*.png` - 测试图像
  - `2_dog/param/park_*.json` - Ocam内参
  - `../../r_calib/data/trans/camera_extrinsics_ikalibr.json5` - 外参

- **文档**：
  - `FIXES_SUMMARY.md` - 详细修复总结
  - `test_ocam_debug.sh` - 自动化测试脚本

## 进一步优化

如果结果仍不理想，可以：

1. **调整搜索范围**：
   在`src/calibration.cpp`中修改搜索边界。

2. **调整tail大小**：
   在`src/optimizer.cpp:512-518`中调整各相机的tail大小。

3. **调整纹理提取**：
   在`src/calibration.cpp:464-557`中调整纹理提取参数。

4. **调整BEV分辨率**：
   在`src/calibration.cpp:404`中修改`bev_rows`和`bev_cols`。

## 联系信息

如有问题，请参考：
- 算法文档：`IMPLEMENTATION_SUMMARY.md`（如果存在）
- Ocam使用说明：`OCAM_USAGE.md`（如果存在）
- Python参考实现：`../../py/ocam_disorder.py`
