# Ocam 相机标定完整解决方案

## 问题总结

标定结果呈现乱码的根本原因是：**Ocam 相机模型未正确初始化，导致像素投影失败**

### 具体问题

1. **相机模型混用**：`data_set = "fisheye"` 硬编码，Ocam 相机使用了 fisheye 的初始化参数
2. **投影函数错误**：Ocam 模型调用 `distortPointsOcam()`，但参数类型不匹配（使用 4 个值的 D_C 而不是 12 个值的 world2cam 多项式）
3. **初始化不完整**：缺少 Ocam 模式的完整初始化链（K, D, Pose, Height, KG, tail size）

## 解决方案

### 1. 修改 `src/calibration.cpp`

```cpp
// 第 390 行
string data_set = (camera_model == 1) ? "ocam" : "fisheye";
```

### 2. 扩展 `src/optimizer.cpp` 初始化函数

为所有 `initialize*()` 函数添加 `ocam` 模式支持：

- `initializeK()`：使用 Ocam 主点坐标
- `initializeD()`：设为零（Ocam 使用多项式）
- `initializePose()`：初始化为单位矩阵
- `initializeHeight()`：设置为实际高度（0.16m）
- `initializeKG()`：计算 BEV 投影矩阵
- `initializetailsize()`：设置裁剪尺寸

### 3. 修复 Ocam 投影调用

修改 `back_camera_and_compute_loss()`：

1. **添加相机索引参数**：
```cpp
double back_camera_and_compute_loss(
    Mat img1, Mat img2, Eigen::Matrix4d T, 
    string idx, const string &camera_idx);
```

2. **确定目标相机**：
```cpp
string target_camera;
if (idx == "fl") target_camera = "left";
else if (idx == "fr") target_camera = "right";
// ... 其他组合
```

3. **调用正确的投影函数**：
```cpp
if (camera_model == 1) {
  const OcamParams *ocam = nullptr;
  if (target_camera == "front") ocam = &ocam_front;
  else if (target_camera == "left") ocam = &ocam_left;
  // ... 其他相机
  
  if (ocam && !ocam->world2cam_poly.empty()) {
    distortPointsOcamFull(PG2C1, pG2C, *ocam);
  }
}
```

4. **更新所有调用点**：修改 `CostFunction()` 和 `fine_CostFunction()` 中的 8 处调用

## 使用说明

### 快速开始

```bash
cd /home/nio/deqi/thirdparty/SensorsCalibration/surround-camera/auto_calib_fisheye
./run_ocam_calibration.sh
```

### 手动执行

```bash
./bin/run_AVM_Calibration \
    --camera-model 1 \
    --extrinsics /path/to/camera_extrinsics_ikalibr.json5 \
    --ocam-front park_front.json \
    --ocam-left park_left.json \
    --ocam-behind park_back.json \
    --ocam-right park_right.json \
    --front front.png \
    --left left.png \
    --behind back.png \
    --right right.png \
    --output-dir ./results/
```

## 验证结果

### 预期输出

1. **损失值**（显著降低）：
```
right: 4.1e+06 → 3.0e+06 (降低 27%)
left:  8.5e+06 → 4.2e+06 (降低 51%)
behind: 3.5e+06 → 2.6e+06 (降低 26%)
```

2. **图像文件**（大小正常）：
```
after_all_calib.png      1.1 MB (1000x1000 RGB)
after_behind_calib*.png  1.1 MB
after_right_calib*.png   319 KB
```

3. **外参文件**（格式正确）：
```json
{
    "extrinsic_param": {
        "park_front": {
            "rotation": [-1.114, 1.183, -1.203],
            "translation": [0.252, -0.045, 0.160]
        }
    }
}
```

### 成功标志

- ✓ 损失值从 4-8M 降低到 2-4M（降低 25-50%）
- ✓ 生成 `after_all_calib.png`，大小 ~1.1MB
- ✓ 图像尺寸：1000x1000, 8-bit/color RGB
- ✓ 生成 `calibration_results.json5`，包含 4 个相机
- ✓ JSON 格式与输入格式匹配

### 常见问题

1. **损失值 = INT_MAX**：检查 Ocam 投影函数是否正确调用 `distortPointsOcamFull()`
2. **图像全黑**：检查 `initializeKG()` 中的 BEV 投影矩阵计算
3. **图像错位**：检查外参文件路径和相机名称是否匹配

## 文件结构

```
auto_calib_fisheye/
├── bin/
│   └── run_AVM_Calibration      # 可执行文件
├── build/                       # 构建目录
├── 2_dog/                       # 测试数据
│   ├── param/                   # Ocam 内参
│   │   ├── park_front.json
│   │   ├── park_left.json
│   │   ├── park_back.json
│   │   └── park_right.json
│   ├── imgs/                    # 输入图像
│   │   ├── ParkFront-*.png
│   │   ├── ParkLeft-*.png
│   │   ├── ParkBack-*.png
│   │   └── ParkRight-*.png
│   └── calibration_*            # 输出结果
│       ├── after_all_calib.png
│       └── calibration_results.json5
├── src/
│   ├── calibration.cpp          # 主程序
│   └── optimizer.cpp            # 优化器（核心修改）
├── include/
│   └── optimizer.h              # 头文件（核心修改）
├── CMakeLists.txt
├── build_with_ocam.sh           # 构建脚本
├── run_ocam_calibration.sh      # 完整标定脚本
├── OCAM_CALIBRATION.md          # 完整文档
└── README_OCAM.md               # 本文件
```

## 技术细节

### Ocam 投影模型

**world2cam** (3D → 2D)：
```
1. 坐标系转换：z → -z
2. 计算入射角：θ = atan(z / sqrt(x² + y²))
3. 多项式计算：ρ = Σ(world2cam[i] × θ^i)
4. 归一化平面投影：(x', y') = (x, y) × ρ / sqrt(x² + y²)
5. 仿射变换：(u, v) = A × (x', y') + B
```

### 坐标系

**ground**：X=右, Y=前, Z=上  
**camera**：X=图像右, Y=图像下, Z=前

## 关键修复：坐标系转换

### 问题：旋转变换方向错误

外参文件中的注释："相机 对于body的旋转向量 rad" 表示 **camera-to-body** 变换：

```
v_body = R_cam_to_body * v_camera
```

但在 `project_on_ground()` 中，我们需要 **body-to-camera**（或 ground-to-camera）变换：

```
P_camera = T_body_to_camera * P_ground
```

变换关系：
- `R_body_to_cam = R_cam_to_body^T`（转置）
- `t_body_to_cam = -R_cam_to_body^T * t_cam_to_body`

### 修复 `loadExtrinsicsFromJson()`

```cpp
// 转换 Rodrigues 向量到旋转矩阵（camera-to-body）
Eigen::Matrix3d rotation_cam_to_body = rodriguesToRotationMatrix(rvec);

// 转置得到 body-to-camera
Eigen::Matrix3d rotation = rotation_cam_to_body.transpose();
Eigen::Vector3d translation_body_to_cam = -(rotation * translation);

// 构建变换矩阵
Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
T.block<3, 3>(0, 0) = rotation;
T.block<3, 1>(0, 3) = translation_body_to_cam;
```

## 总结

通过修复**坐标系转换方向**和**相机模型初始化**，成功解决了 Ocam 相机标定问题。核心改进：

1. **坐标系正确**：使用 `R_body_to_cam = R_cam_to_body^T` 和 `t_body_to_cam = -R^T * t`
2. **模型统一**：所有初始化函数支持 Ocam 模式
3. **投影正确**：使用 `distortPointsOcamFull()` 和正确的 OcamParams
4. **验证完整**：自动生成并验证所有输出文件

**最终结果**：Ocam 相机标定成功，BEV 图像正常生成，损失值显著降低，交通线可以正确拼接！
