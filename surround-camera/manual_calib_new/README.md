# AVM Manual Calibration (New Format) - OPTIMIZED

基于 `auto_calib_fisheye` 重写的手动标定工具，适配新的相机参数格式和 BEV 投影方式。

## 优化内容 (2024-03-28)

### 1. 分辨率提升
- **BEV 尺寸**: 1000x1000 → **2000x2000** (4倍像素)
- **像素精度**: 1cm/pixel → **0.5cm/pixel** (2倍精度)
- **显示窗口**: 1000x1000 → **1200x1200**
- **输出图像**: stitching.png 现在为 2000x2000 高分辨率

### 2. 中心黑色区域减小
- **中心盲区**: 200x200 → **100x100** (减少75%)
- **显示更多有效内容**

### 3. 边框阴影减小
- **Tail 裁剪**: 300px → **150px** (减少50%)
- **融合边界**: 400px → **150px** margin
- **更少的黑色边框，更多的图像内容**

### 4. 融合逻辑优化
- 动态计算融合区域
- 改进角落融合算法
- 更清晰的相机区域分离

## 主要改进

### 1. 内参格式支持

#### Ocam 模型 (camera_model=1, 默认)
- 使用多项式畸变模型
- 参数文件格式：
```json
{
  "intrinsic_param": {
    "camera_width": 1920,
    "camera_height": 1080,
    "principal_point": [959.5, 539.5],
    "affine_c": 0.999,
    "affine_d": 0.001,
    "affine_e": 0.001,
    "cam2world": [1.0, 0.0, 0.0, ...],
    "world2cam": [1.0, 0.0, 0.0, ...]
  }
}
```

#### Pinhole/Fisheye 模型 (camera_model=0,2)
- 保持与传统格式兼容
- K 矩阵 + 畸变系数

### 2. 外参格式支持

新的外参文件格式：
```json
{
  "extrinsic_param": {
    "park_front": {
      "rotation": [rx, ry, rz],
      "translation": [tx, ty, tz]
    },
    "park_left": {...},
    "park_back": {...},
    "park_right": {...}
  }
}
```

**重要**: 外参文件包含的是 **camera-to-body** 变换，程序会自动转换为 **body-to-camera** (T_vehicle_to_camera)。

### 3. BEV 投影方式

采用 CPAC 的 BEV 投影公式：
- 车身坐标系：x=forward, y=left, z=up
- BEV 像素尺寸：1000x1000
- 分辨率：1cm/pixel (metric_ratio_m = 0.01)
- 轮距：3.01m (可配置)

投影流程：
1. 生成 BEV 地面点 (车身坐标系)
2. 使用外参变换到相机坐标系
3. 使用 Ocam/Pinhole 模型投影到图像平面
4. OpenCV remap 生成 BEV 图像

## 编译

```bash
cd manual_calib_new
mkdir -p build && cd build
cmake ..
make -j4
```

## 使用方法

### 基本用法

```bash
./bin/run_avm_new <image_path> <param_path> <extrinsic_json> [camera_model]
```

### 示例

**Ocam 模型：**
```bash
./bin/run_avm_new \
  ./test_data/imgs \
  ./test_data/ocam_params \
  ./test_data/extrinsics.json \
  1
```

**Pinhole 模型：**
```bash
./bin/run_avm_new \
  ./test_data/imgs \
  ./test_data/intrinsics \
  ./test_data/extrinsics.json \
  2
```

### 文件结构

```
./imgs/
  ├── Front.png
  ├── Left.png
  ├── Back.png
  └── Right.png

./ocam_params/ (Ocam 模型)
  ├── park_front.json
  ├── park_left.json
  ├── park_back.json
  └── park_right.json

./extrinsics.json (新格式)
```

## GUI 操作

### 控制面板

- **stitching**: 切换拼接模式/单相机模式
- **deg step**: 角度调整步长 (度)
- **t step**: 平移调整步长 (cm)
- **frame**: 选择当前调整的相机 (0=front, 1=left, 2=back, 3=right)

### 校准按钮

- **+/- x/y/z deg**: 绕各轴旋转调整
- **+/- x/y/z trans**: 沿各轴平移调整
- **Reset**: 重置当前相机外参
- **Save**: 保存标定结果

### 键盘快捷键

- `q/a`: +/- X 轴旋转
- `w/s`: +/- Y 轴旋转
- `e/d`: +/- Z 轴旋转
- `r/f`: +/- X 轴平移
- `t/g`: +/- Y 轴平移
- `y/h`: +/- Z 轴平移 (同时调整高度)

## 输出文件

### calibration_X.txt
保存每个相机的标定结果：
- 旋转矩阵 R (3x3)
- 平移向量 t
- 相机高度
- JSON 格式外参矩阵 (4x4)

### stitching.png
拼接的环绕视图图像 (拼接模式下保存)

### calibimg_X.png
当前相机的 BEV 图像 (单相机模式下保存)

## 与旧版本对比

| 特性 | 旧版 (run_avm) | 新版 (run_avm_new) |
|------|---------------|-------------------|
| 内参格式 | 传统 K + dist | Ocam 多项式 / 传统格式 |
| 外参格式 | 直接 4x4 矩阵 | rotation + translation |
| 坐标系 | 自定义 | CPAC 标准车身坐标系 |
| BEV投影 | 简单投影 | CPAC 公式 |
| Ocam支持 | 部分 | 完整 |
| 高度处理 | 固定值 | 从外参自动计算 |

## 依赖

- OpenCV 4.x
- Pangolin
- Eigen3
- Boost
- JsonCpp
