# 前视3相机手动标定工具 (Front 3 Cameras Manual Calibration)

## 功能说明

本工具用于手动标定前视3相机系统（左前鱼眼 + 前视Ocam + 右前鱼眼），实现：
- 鱼眼相机去畸变（OpenCV fisheye 4参数模型）
- Ocam相机去畸变（多项式模型）
- 透视变换 + 融合拼接
- 6自由度外参手动微调

## 编译步骤

```bash
cd third_party/SensorsCalibration/surround-camera/manual_calib_front3
mkdir -p build && cd build
cmake ..
make -j4
```

## 依赖库

- OpenCV (3.x 或 4.x)
- Pangolin (用于GUI)
- Eigen3
- Boost::filesystem
- JsonCpp

## 输入文件格式

### 1. 图像文件
放置于指定目录，命名格式：
- `front_left.png` - 左前鱼眼相机图像
- `front.png` - 前视Ocam相机图像
- `front_right.png` - 右前鱼眼相机图像

### 2. 内参文件

#### 鱼眼相机 (front_left.json / front_right.json)
```json
{
  "name": "front_left",
  "intrinsic_param": {
    "camera_width": 3848,
    "camera_height": 2168,
    "camera_matrix": [
      [fx, 0, cx],
      [0, fy, cy],
      [0, 0, 1]
    ],
    "distortion_coeffcients": [k1, k2, k3, k4],
    "distortion_model": "opencv-fisheye elements"
  }
}
```

#### Ocam相机 (park_front.json)
```json
{
  "name": "park_front",
  "intrinsic_param": {
    "camera_model": 2,
    "camera_width": 1920,
    "camera_height": 1536,
    "principal_point": [cx, cy],
    "affine_c": 0.9988,
    "affine_d": 0.00048,
    "affine_e": -0.00079,
    "world2cam": [a0, a1, a2, ..., a11],
    "world2cam_len": 12,
    "distortion_model": "polynomial coefficients"
  }
}
```

### 3. 外参文件 (extrinsics.json)
```json
{
  "extrinsic_param": {
    "front_left": {
      "rotation": [rx, ry, rz],
      "translation": [tx, ty, tz]
    },
    "park_front": {
      "rotation": [rx, ry, rz],
      "translation": [tx, ty, tz]
    },
    "front_right": {
      "rotation": [rx, ry, rz],
      "translation": [tx, ty, tz]
    }
  }
}
```
- `rotation`: Rodrigues旋转向量 (3x1)
- `translation`: 相机在车身坐标系中的位置 (单位: 米)

## 运行方法

```bash
./run_front3_calib <图像目录> <内参目录> <外参文件>

# 示例
./run_front3_calib ./imgs ./intrinsics ./extrinsics.json
```

## GUI操作说明

### 控制面板
- **frame**: 选择当前标定的相机 (0=FL, 1=F, 2=FR)
- **deg step**: 旋转步长 (度)
- **t step(cm)**: 平移步长 (厘米)

### 6自由度调整按钮
- **+/- x deg**: 绕X轴旋转 (Roll)
- **+/- y deg**: 绕Y轴旋转 (Pitch)
- **+/- z deg**: 绕Z轴旋转 (Yaw)
- **+/- x trans**: X方向平移
- **+/- y trans**: Y方向平移
- **+/- z trans**: Z方向平移

### 键盘快捷键
| 按键 | 功能 |
|-----|------|
| q/a | +/- X旋转 |
| w/s | +/- Y旋转 |
| e/d | +/- Z旋转 |
| r/f | +/- X平移 |
| t/g | +/- Y平移 |
| y/h | +/- Z平移 |

### 功能按钮
- **Reset**: 重置当前相机外参为初始值
- **Save**: 保存所有相机标定结果

## 输出文件

1. **calibration_front_left.txt** - 左前相机标定结果
2. **calibration_front.txt** - 前视相机标定结果
3. **calibration_front_right.txt** - 右前相机标定结果
4. **front3_extrinsics_calibrated.json** - 完整外参JSON文件
5. **front3_stitched.png** - 拼接结果图像
6. **front3_calibration_metrics.txt** - 标定质量评估报告 (新增)

## 输出格式说明

程序内部使用 `T_body_to_cam`（车身到相机变换），输出格式为 `T_cam_to_body`（相机到车身变换），与输入格式一致。

```json
{
  "extrinsic_param": {
    "front_left": {
      "rotation": [rx, ry, rz],
      "translation": [tx, ty, tz]
    },
    "park_front": {
      "rotation": [rx, ry, rz],
      "translation": [tx, ty, tz]
    },
    "front_right": {
      "rotation": [rx, ry, rz],
      "translation": [tx, ty, tz]
    }
  }
}
```

## 标定质量评估指标

点击 **Save** 按钮后，系统会自动计算以下客观指标：

| 指标 | 说明 | 优秀标准 |
|-----|------|---------|
| Photometric Loss | 重叠区域像素灰度差异 (RMSE) | < 5 |
| SSIM | 结构相似性指数 [-1, 1] | > 0.9 |
| Edge Alignment | 边缘对齐度 (IoU) | > 0.75 |
| Feature Matching | 特征点匹配内点数量 | > 50 |

### 评估结果
- **综合得分**: 0-100分
- **等级**: Excellent (>80), Good (60-79), Fair (40-59), Poor (<40)
- **输出文件**: `front3_calibration_metrics.txt`

详细说明请参考 `METRICS_GUIDE.md`

## 参考文件

- 鱼眼内参示例: `../manual_calib_new/2_dog/param/front_left.json`
- Ocam内参示例: `../manual_calib_new/2_dog/param/park_front.json`
