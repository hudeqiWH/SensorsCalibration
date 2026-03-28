# Ocam 相机标定完整指南

## 问题概述

标定结果呈现乱码，主要原因是：

1. **坐标系转换错误**：ground->camera 与 camera->ground 混淆
2. **相机模型混用**：Ocam 内参与 fisheye 外参混用
3. **初始化参数错误**：未正确处理 Ocam 模式的所有初始化

## 核心问题分析

### 问题 1：相机模型混用

**症状**：`data_set = "fisheye"` 硬编码，导致 Ocam 相机使用了 fisheye 的初始化参数

**影响**：
- 错误的内参矩阵 K
- 错误的畸变参数 D
- 错误的相机高度
- 错误的 BEV 投影矩阵 KG

**修复**：
```cpp
// src/calibration.cpp
string data_set = (camera_model == 1) ? "ocam" : "fisheye";
```

然后在所有 `initialize*()` 函数中添加 `ocam` 模式：
- `initializeK()` - 使用 Ocam 主点坐标
- `initializeD()` - 设为零（Ocam 使用多项式）
- `initializePose()` - 初始化为单位矩阵
- `initializeHeight()` - 设置为实际高度（约 0.16m）
- `initializeKG()` - 计算 BEV 投影矩阵
- `initializetailsize()` - 设置裁剪尺寸

### 问题 2：Ocam 投影函数未正确调用

**症状**：损失值 = INT_MAX (2.14748e+09)，几乎所有像素投影失败

**原因**：在 `back_camera_and_compute_loss()` 中，Ocam 模型调用 `distortPointsOcam()`，但该函数使用传统的 `vector<double> &D_C` 参数（4个值），而不是 Ocam 的 `world2cam` 多项式（12个值）

**影响**：
- 像素投影到错误位置
- 超过30个像素失败时返回 INT_MAX
- 优化算法无法收敛

**修复**：
1. 修改 `back_camera_and_compute_loss()` 签名，添加 `camera_idx` 参数
2. 根据 `idx` 确定目标相机（front/left/right/behind）
3. 调用 `distortPointsOcamFull()` 并传递正确的 `OcamParams`
4. 在 `CostFunction()` 和 `fine_CostFunction()` 中传递相机索引

```cpp
// 修改后
double Optimizer::back_camera_and_compute_loss(
    Mat img1, Mat img2, Eigen::Matrix4d T, 
    string idx, const string &camera_idx)

// 使用
double loss = back_camera_and_compute_loss(
    imgf_bev, imgr_gray, Tr, "fr", "right");
```

### 问题 3：坐标系定义不清

代码注释为 `// ground->camera`，但实际使用中可能是 `camera->ground`。

在 `project_on_ground()` 中：
```cpp
P_GC = T_CG_ * P_G;  // T_CG * P_G
```

这表示：`P_camera = T_camera_from_ground * P_ground`

所以 T_CG 确实是 ground->camera 变换。

### 问题 3：初始化不完整

Ocam 相机需要完整的初始化链：
- `initializeK()` - 内参矩阵
- `initializeD()` - 畸变参数
- `initializePose()` - 外参初始值
- `initializeKG()` - BEV 投影矩阵
- `initializeHeight()` - 相机高度
- `initializetailsize()` - 裁剪尺寸

## 解决方案

### 修复 1：添加 Ocam 模式支持

在 `src/calibration.cpp` 中：
```cpp
string data_set = (camera_model == 1) ? "ocam" : "fisheye";
```

### 修复 2：坐标系转换

#### 问题：旋转变换方向错误

外参文件中的旋转向量是 **camera-to-body**（相机到车体）变换：

```
v_body = R_cam_to_body * v_camera
```

但在 `project_on_ground()` 中，我们需要 **body-to-camera**（车体到相机）变换：

```
P_camera = T_body_to_camera * P_ground
```

其中：
- `R_body_to_cam = R_cam_to_body^T`
- `t_body_to_cam = -R_cam_to_body^T * t_cam_to_body`

#### 修复 `loadExtrinsicsFromJson()`

```cpp
void Optimizer::loadExtrinsicsFromJson(const string &filename) {
  // ... 读取 rvec 和 translation ...

  // 转换 Rodrigues 向量到旋转矩阵（camera-to-body）
  Eigen::Matrix3d rotation_cam_to_body = rodriguesToRotationMatrix(rvec);

  // 转置得到 body-to-camera
  Eigen::Matrix3d rotation = rotation_cam_to_body.transpose();
  Eigen::Vector3d translation_body_to_cam = -(rotation * translation);

  // 构建变换矩阵
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3, 3>(0, 0) = rotation;
  T.block<3, 1>(0, 3) = translation_body_to_cam;
}
```

### 修复 3：扩展初始化函数

#### `initializeHeight()` - 相机高度
```cpp
else if (data_index == "ocam") {
  // 根据外参文件设置（约 0.16m）
  hf = 0.16;
  hl = 0.16;
  hb = 0.16;
  hr = 0.16;
}
```

#### `initializeKG()` - BEV 投影矩阵
```cpp
else if (data_index == "ocam") {
  double ground_resolution = 0.01; // 1cm per pixel
  K_G(0, 0) = 1.0 / ground_resolution;
  K_G(1, 1) = -1.0 / ground_resolution; // negative for image y-down
  K_G(0, 2) = bcols / 2.0;
  K_G(1, 2) = brows / 2.0;
  K_G(2, 2) = 1.0;
}
```

## 使用指南

### 快速开始

```bash
cd /home/nio/deqi/thirdparty/SensorsCalibration/surround-camera/auto_calib_fisheye

# 运行完整标定
./run_calibration_with_extrinsics.sh
```

### 手动执行

```bash
# 创建输出目录
mkdir -p ./results

# 运行标定
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

### 验证结果

运行完整标定流程：

```bash
cd /home/nio/deqi/thirdparty/SensorsCalibration/surround-camera/auto_calib_fisheye
./run_ocam_calibration.sh
```

**预期输出**：

1. **损失值**（应显著降低）：
```
luminorsity loss before pre opt: 4.1e+06
luminorsity loss after pre opt:  3.0e+06  (降低 ~27%)
luminorsity loss after opt:      3.0e+06  (降低 ~27%)
```

2. **图像文件**（文件大小正常）：
```
-rw-rw-r-- 1.1M after_all_calib.png       # 最终 BEV 拼接图
-rw-rw-r-- 1.1M after_behind_calib1.png   # Behind 相机优化过程
-rw-rw-r-- 319K after_right_calib1.png    # Right 相机优化过程
```

3. **外参文件**（JSON5 格式）：
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

**检查清单**：
- ✓ 损失值从 4-8M 降低到 2-4M（降低 25-50%）
- ✓ 生成 `after_all_calib.png`，大小 ~1.1MB
- ✓ 图像尺寸：1000x1000, 8-bit/color RGB
- ✓ 生成 `calibration_results.json5`，包含 4 个相机
- ✓ JSON 格式正确，与输入格式匹配

**故障排除**：
- **损失值 = INT_MAX**：检查 Ocam 投影函数是否正确调用
- **图像全黑**：检查 KG 矩阵和相机高度
- **图像错位**：检查外参文件和相机名称

## 参数调整

### 如果 BEV 图像太小/太大

修改 `initializeKG()` 中的 `ground_resolution`：
```cpp
double ground_resolution = 0.01; // 减小值放大图像，增大值缩小图像
```

### 如果图像位置不对

检查外参文件格式：
```json
{
    "extrinsic_param": {
        "park_front": {
            "rotation": [-1.242, 1.195, -1.206],
            "translation": [0.258, -0.048, 0.160]
        }
    }
}
```

注意：
- `rotation`：Rodrigues 向量（弧度）
- `translation`：平移向量（米）
- 相机名称必须匹配：`park_front`, `park_left`, `park_back`, `park_right`

### 如果搜索不收敛

修改搜索范围（在 `calibration.cpp` 中）：
```cpp
// 减小搜索范围
threads[0] = thread(&Optimizer::Calibrate_right, &opt, iter_nums, -1, 1, -1, 1,
                    -1, 1, -0.005, 0.005, -0.005, 0.005, -0.005, 0.005);
```

## 故障排除

### 问题：图像全黑
**原因**：KG 矩阵错误，所有点投影到图像外
**解决**：检查 `initializeKG()` 的 Ocam 分支

### 问题：图像错位
**原因**：外参错误或高度错误
**解决**：
1. 验证外参文件路径
2. 检查相机名称是否匹配
3. 调整 `initializeHeight()` 的值

### 问题：优化不收敛
**原因**：初始值太差或搜索范围太大
**解决**：
1. 确保加载了正确的外参文件
2. 减小搜索范围
3. 增加迭代次数

### 问题：编译错误
**原因**：jsoncpp 未正确链接
**解决**：
```bash
sudo apt-get install libjsoncpp-dev
cd build && cmake .. && make -j$(nproc)
```

## 文件结构

```
auto_calib_fisheye/
├── bin/
│   └── run_AVM_Calibration      # 可执行文件
├── build/                       # 构建目录
├── 2_dog/
│   ├── param/
│   │   ├── park_front.json      # Ocam 内参
│   │   ├── park_left.json
│   │   ├── park_back.json
│   │   └── park_right.json
│   ├── imgs/
│   │   ├── ParkFront-*.png      # 输入图像
│   │   ├── ParkLeft-*.png
│   │   ├── ParkBack-*.png
│   │   └── ParkRight-*.png
│   └── results/                 # 输出结果
│       ├── before_all_calib.png
│       ├── after_all_calib.png
│       └── calibration_results.json5
├── src/
│   ├── calibration.cpp          # 主程序
│   └── optimizer.cpp            # 优化器
├── include/
│   └── optimizer.h
├── CMakeLists.txt
├── build_with_ocam.sh           # 构建脚本
├── run_calibration_with_extrinsics.sh  # 标定脚本
└── OCAM_CALIBRATION.md          # 本文档
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

**cam2world** (2D → 3D)：
```
逆向使用 cam2world 多项式
```

### 坐标系定义

**ground 坐标系**：
- X：车辆右侧
- Y：车辆前方
- Z：地面向上

**camera 坐标系**：
- X：图像右
- Y：图像下
- Z：光轴向前

**变换关系**：
```
P_camera = T_camera_from_ground × P_ground
```

### 参数加载顺序

```
1. 构造函数调用：
   - initializeK()
   - initializeD()
   - initializePose()      // 设置初始值
   - initializeKG()
   - initializeHeight()
   - initializetailsize()

2. 加载 Ocam 参数：
   - loadOcamParams()      // 加载内参

3. 加载外参：
   - loadExtrinsicsFromJson()  // 覆盖初始值

4. 生成初始 BEV：
   - project_on_ground()   // 使用加载的参数
```

## 性能优化

### 编译优化
```bash
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)
```

### 运行优化
- 使用 Release 模式（默认）
- 调整线程数（默认 7 线程）
- 减少迭代次数（修改 `iter_nums`）

## 参考文献

1. Scaramuzza, et al. "A Toolbox for Easily Calibrating Omnidirectional Cameras"
2. `ocam_disorder.py` - Python 参考实现
3. iKalibr 文档 - 外参格式规范

---

**注意**：本文档整合了所有之前的文档和脚本信息，提供了完整的 Ocam 相机标定指南。
