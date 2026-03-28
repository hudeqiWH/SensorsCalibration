# CPAC与SensorsCalibration差异分析报告

## 1. 核心差异

### 1.1 外参使用方式差异

#### CPAC (nt2_generate_bev.py)
```python
# 外参文件包含完整的T_camera_to_vehicle
cam_pt = T_vehicle_to_camera @ veh_pt  # 车辆坐标→相机坐标
```

#### SensorsCalibration (当前)
```cpp
// 外参文件包含：camera-to-body
// 我们计算：body-to-camera (invert)
T_body_to_cam = T_cam_to_body.inverse();
P_cam = T_body_to_cam * P_body;
```

**关键问题**：当前的外参转换逻辑与cpac不一致。

### 1.2 图像旋转处理

#### CPAC
- **无显式旋转**：图像直接读取，标定参数包含安装方向
- 相机命名：`svc_front`, `svc_left`, `svc_rear`, `svc_right`

#### SensorsCalibration
- **手动旋转**：
  - front: ROTATE_180
  - left: ROTATE_90_COUNTERCLOCKWISE
  - right: ROTATE_90_CLOCKWISE
  - back: ROTATE_180
- 相机命名：`park_front`, `park_left`, `park_back`, `park_right`

**问题**：手动旋转后，图像坐标系发生变化，但外参未相应调整。

### 1.3 标定文件结构

#### CPAC
```
data_dir/
├── camera/
│   ├── svc_front/
│   │   └── image/
│   │       └── svc_front.jpg
│   ├── svc_left/
│   └── ...
├── lidar/
└── vehicle_info/
    └── vehicle_info.json
```

#### SensorsCalibration
```
2_dog/
├── imgs/
│   ├── ParkFront-xxx.png
│   ├── ParkLeft-xxx.png
│   └── ...
└── param/
    ├── park_front.json
    └── ...
```

### 1.4 BEV参数

#### CPAC
- `bev_width=2200`, `bev_height=1800`
- `metric_ratio=1.0` (cm/pixel)
- `wheel_base` 从车辆信息读取
- `car_width`, `car_length` 从车辆信息读取

#### SensorsCalibration
- `bev_rows=1000`, `bev_cols=1000`
- `ground_resolution=0.02` (m/pixel = 2cm/pixel)
- 车辆尺寸硬编码：`sizef=340`, `sizel=390`, etc.

### 1.5 多项式求值

#### CPAC
```python
# 使用np.poly1d，系数从高次到低次
f = np.poly1d(self.invpol[::-1])
rho = f(theta)
```

#### SensorsCalibration (优化后)
```cpp
// 预计算系数，从高次到低次
Eigen::VectorXd world2cam_coeffs;
// ... 加载时填充 ...

// Horner方法求值
double rho = 0.0;
for (int k = 0; k < coeffs.size(); k++) {
  rho = rho * theta + coeffs(k);
}
```

### 1.6 坐标系定义

#### CPAC
- **车辆坐标系**：x前，y左，z上
- **相机坐标系**：x右，y下，z前（相机朝向）
- **地面坐标系**：z=0平面

#### SensorsCalibration
- **body坐标系**：x前，y左，z上
- **相机坐标系**：x右，y下，z前
- **地面坐标系**：z=0平面

## 2. 问题分析

### 2.1 外参转换错误

**当前代码**（错误）：
```cpp
// extrinsics文件: camera-to-body
// 我们计算: body-to-camera
Eigen::Matrix3d R_cam_to_body = rodriguesToRotationMatrix(rvec);
Eigen::Matrix3d R_body_to_cam = R_cam_to_body.transpose();
Eigen::Vector3d t_body_to_cam = -R_body_to_cam * translation_vec;
```

**正确逻辑**（应改为）：
```cpp
// extrinsics文件: camera-to-body
// 需要: body-to-camera
// 如果extrinsics是T_cam_to_body，那么T_body_to_cam = T_cam_to_body.inverse()
// 但注意：translation_vec是相机在body坐标系中的位置
// 所以: P_body = R_cam_to_body.T * (P_cam - t_cam_in_body)
// 或:   P_cam = R_cam_to_body * P_body + t_cam_in_body

// 实际上，应该这样计算：
Eigen::Matrix3d R_cam_to_body = rodriguesToRotationMatrix(rvec);
Eigen::Matrix3d R_body_to_cam = R_cam_to_body.transpose();
Eigen::Vector3d t_cam_in_body = translation_vec;
Eigen::Vector3d t_body_to_cam = -R_body_to_cam * t_cam_in_body;

// 或者更直接：使用完整的逆变换
Eigen::Matrix4d T_cam_to_body = Eigen::Matrix4d::Identity();
T_cam_to_body.block<3, 3>(0, 0) = R_cam_to_body;
T_cam_to_body.block<3, 1>(0, 3) = t_cam_in_body;

Eigen::Matrix4d T_body_to_cam = T_cam_to_body.inverse();
```

### 2.2 图像旋转不匹配

**问题**：手动旋转图像后，像素坐标系发生变化，但投影计算使用的内参没有相应调整。

**解决方案**：
- 选项1：不手动旋转图像，让外参包含旋转
- 选项2：旋转图像后，相应调整内参（cx, cy）和投影结果

### 2.3 KG参数可能不合适

当前使用`ground_resolution=0.02`（2cm/像素），这可能与cpac的`metric_ratio=1.0`（1cm/像素）不一致。

## 3. 移植方案

### 3.1 外参转换修正

修改`loadExtrinsicsFromJson`：
```cpp
// 外参文件: camera-to-body
// 我们需要: body-to-camera for projection
Eigen::Matrix3d R_cam_to_body = rodriguesToRotationMatrix(rvec);
Eigen::Vector3d t_cam_in_body = translation_vec;

// Build T_cam_to_body
Eigen::Matrix4d T_cam_to_body = Eigen::Matrix4d::Identity();
T_cam_to_body.block<3, 3>(0, 0) = R_cam_to_body;
T_cam_to_body.block<3, 1>(0, 3) = t_cam_in_body;

// Invert to get body-to-camera
Eigen::Matrix4d T_body_to_cam = T_cam_to_body.inverse();

extrinsic_front = T_body_to_cam;
```

### 3.2 移除图像旋转

修改`test_bev_simple.cpp`：
```cpp
// 移除这些旋转操作
// rotate(imgf, imgf, ROTATE_180);
// rotate(imgl, imgl, ROTATE_90_COUNTERCLOCKWISE);
// rotate(imgr, imgr, ROTATE_90_CLOCKWISE);
// rotate(imgb, imgb, ROTATE_180);
```

### 3.3 调整KG参数

修改`initializeKG`：
```cpp
// 改为与cpac一致：1cm/pixel
double ground_resolution = 0.01;  // 0.01 m/pixel = 1cm/pixel
```

### 3.4 添加车辆尺寸参数

在`initializeHeight`或`initializePose`中：
```cpp
// 从外参文件或车辆信息读取
veh_width = 1.96;      // meters
veh_length = 4.90;     // meters
wheel_base = 3.01;     // meters
```

### 3.5 坐标系验证

添加调试输出：
```cpp
// 验证坐标系
Eigen::Vector4d test_ground = Eigen::Vector4d(0, 10, 0, 1);  // 10m forward
Eigen::Vector4d test_cam = T_body_to_cam * test_ground;

// 应该得到：
// x: ~0 (或在相机视野内)
// y: ~camera_height (相机高度)
// z: positive (在相机前方)
```

## 4. 验证步骤

### 4.1 单相机验证

对于front相机：
- 图像中地面点应投影到图像下半部分
- 远处点（y=10m）应该在图像底部附近
- 近处点（y=2m）应该在图像顶部附近

### 4.2 环绕视图验证

- front和back相机的重叠区域应对齐
- left和right相机的重叠区域应对齐
- 车辆轮廓应正确显示

## 5. 需要移植的CPAC特性

### 5.1 工厂标定加载

```python
# 支持从多个目录加载
calibration_param = loader.load_factory_calib(image_dir, lidar_dir, vehicle_info_dir)
```

### 5.2 融合掩码

```cpp
// 使用BlendMask而不是BevMask
self.masks = [BlendMask('front', ...), ...]
```

### 5.3 亮度平衡

```cpp
// 在叠加前进行亮度平衡
if (balance) {
  images = luminance_balance(images);
}
```

## 6. 实施计划

优先级排序：
1. **高**：修正外参转换逻辑（最关键）
2. **高**：移除图像旋转或调整旋转逻辑
3. **中**：调整KG参数（metric_ratio）
4. **中**：添加车辆尺寸参数
5. **低**：移植融合掩码和亮度平衡
6. **低**：支持工厂标定目录结构
