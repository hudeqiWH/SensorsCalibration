# BEV生成测试结果分析

## 当前状态

### 相机外参（加载后）

| 相机 | 旋转(Euler) | 平移(m) | 相机高度 |
|------|-------------|---------|----------|
| Front | -34.8, -88.8, 126.0 | [-0.07, 0.80, -0.25] | 0.80m |
| Left | 0.5, 1.0, 92.6 | [-0.02, 0.81, -0.11] | 0.80m |
| Behind | -73.5, 87.1, 16.5 | [0.03, 0.79, -0.17] | 0.80m |
| Right | -179.6, 3.0, -91.2 | [0.01, 0.81, -0.20] | 0.80m |

### 有效投影点比例

| 相机 | 有效点/总数 | 比例 |
|------|-------------|------|
| Front | 409,391/1,000,000 | 40.9% |
| Left | 541,630/1,000,000 | 54.2% |
| Behind | 331,782/1,000,000 | 33.2% |
| Right | 471,634/1,000,000 | 47.2% |

### 投影像素范围

| 相机 | u范围 | v范围 |
|------|-------|-------|
| Front | [129, 1778] | [713, 1536] |
| Left | [162, 1795] | [820, 1536] |
| Behind | [140, 1784] | [856, 1536] |
| Right | [139, 1767] | [818, 1536] |

## 问题分析

### 1. 有效投影点比例偏低

预期：应该 >80%
实际：33-54%

可能原因：
- BEV坐标范围与相机FOV不匹配
- 车体坐标系与相机坐标系转换有误

### 2. 相机坐标范围异常

前相机（Front）示例：
- x: [-5.13, 5.03] (正常，左右范围)
- y: [0.62, 0.95] (异常，应该是高度方向)
- z: [-3.82, 6.34] (异常，应该是前后方向)

在相机坐标系中：
- y ≈ 0.8m 对应相机高度，这应该是y轴
- z范围[-3.8, 6.3]包含了负值，但CPAC只投影z<0的点

**关键发现**：我的T_CG转换可能有误。

### 3. CPAC坐标系与当前实现对比

CPAC的车体坐标系：
- x: 前方 (forward)
- y: 左方 (left)
- z: 上方 (up)

CPAC的相机坐标系（变换后）：
- z < 0: 相机前方（可见区域）
- x, y: 图像平面

当前相机坐标范围显示：
- y ≈ 0.8 (恒定，对应高度)
- z变化大 (应该是前后方向)

这表明：相机坐标系的y轴是高度，z轴是深度。但CPAC期望z<0为前方。

## 下一步修复方案

### 1. 检查T_body_to_cam是否正确

CPAC的extrinsics加载：
```python
T_vehicle_to_camera = np.eye(4)
T_vehicle_to_camera[:3, :3] = rotation_matrix  # camera-to-body rotation
T_vehicle_to_camera[:3, 3:] = translation      # camera position in body
T_vehicle_to_camera = np.linalg.inv(T_vehicle_to_camera)  # invert
```

我的代码应该与此一致。

### 2. 检查BEV坐标生成

CPAC公式：
```python
veh_x = wheel_base_pixel/2 + bev_height_pixel/2 - j
veh_y = bev_width_pixel/2 - i
veh_pt = [veh_x * metric_ratio/100, veh_y * metric_ratio/100, 0, 1]
```

注意：metric_ratio是cm/pixel，veh_pt需要转换为meters。

如果metric_ratio = 1.0 (cm/pixel) = 0.01 (m/pixel)：
- veh_pt_x = veh_x * 0.01 (meters)

### 3. 可能的修复

1. 检查外参文件中的rotation/translation定义是否与CPAC一致
2. 检查是否需要交换x/y/z轴
3. 检查wheel_base的影响（可能不需要wheel_base偏移）

## 图像观察

从生成的BEV图像看：
- 每个单相机的BEV图显示了地面区域
- 但拼接时四个相机图像不对齐
- 中间有黑色区域（车辆位置）

这表明投影公式基本工作，但坐标系可能整体偏移或旋转错误。
