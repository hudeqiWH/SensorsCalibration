# Ocam相机BEV投影与标定

基于CPAC项目流程移植的Ocam（全向）相机BEV（鸟瞰图）投影实现。

## 快速开始

```bash
# 编译
cd /home/nio/deqi/thirdparty/SensorsCalibration/surround-camera/auto_calib_fisheye
./build_and_test.sh

# 或手动编译
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)

# 运行BEV测试
./bin/test_bev_simple
# 输出: test_bev_output/*.png

# 运行完整标定
./bin/run_AVM_Calibration
```

## 核心实现

### 1. 坐标系定义

**车体坐标系**: X-前方, Y-左方, Z-上方  
**相机坐标系**: X-图像右, Y-图像下, Z-光轴向前 (z>0为前方)

### 2. 外参转换

```cpp
// 外参文件: camera-to-body → 转换为 body-to-camera
Eigen::Matrix4d T_body_to_cam = T_cam_to_body.inverse();
```

### 3. BEV坐标生成

```cpp
double metric_ratio = 0.01;  // 1cm/pixel
veh_x = (wheel_base/2 + height/2 - j) * metric_ratio;  // forward
veh_y = (width/2 - i) * metric_ratio;                  // left
```

### 4. Ocam投影

```cpp
// theta = atan(-z / norm_xy)
double theta = atan(-z / sqrt(x*x + y*y));
// rho = polynomial(theta)
double rho = horner(world2cam_coeffs, theta);
// affine transform
double u = uu + vv*e + cx;
double v = uu*d + vv*c + cy;
```

## 文件结构

```
├── src/optimizer.cpp        # BEV投影实现
├── src/calibration.cpp      # 标定主程序
├── test_bev_simple.cpp      # BEV测试
├── include/optimizer.h      # 头文件
├── 2_dog/param/             # Ocam内参
├── test_bev_output/         # BEV输出
└── README_BEV.md            # 本文件
```

## 参数调整

| 参数 | 位置 | 默认值 | 说明 |
|------|------|--------|------|
| BEV尺寸 | test_bev_simple.cpp | 1000x1000 | 鸟瞰图分辨率 |
| 地面分辨率 | optimizer.cpp | 0.01m/pix | 米/像素 |
| wheel_base | optimizer.cpp | 3.01m | 车辆轴距 |
| 外参文件 | calibration.cpp | camera_extrinsics_ikalibr.json5 | 相机外参 |

## 故障排除

- **图像全黑**: 检查KG矩阵、外参路径、相机高度
- **图像乱码**: 检查外参转换方向、Ocam多项式
- **有效点比例低**: 调整BEV范围或wheel_base

## 参考

- CPAC: `cpac/utils/bev_generation/bev_generator.py`
- Ocam模型: Scaramuzza, et al.
