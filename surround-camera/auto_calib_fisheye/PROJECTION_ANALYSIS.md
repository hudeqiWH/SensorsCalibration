# Ocam投影函数修复完成报告

## 执行摘要

已完成Ocam投影函数的数学修复，添加了缺失的归一化步骤，并验证了计算流程。但BEV图像仍然显示不正确，需要进一步调试地面分辨率参数和输入点范围。

## ✅ 已完成的修复

### 1. Ocam投影函数修复

**文件**: `src/optimizer.cpp:210-263`

**问题**: C++实现缺少归一化步骤
- Python输入是归一化的方向向量（norm=1）
- C++输入是3D点坐标（未归一化）
- 直接使用`sqrt(x*x + y*y)`计算的是点到xy平面的距离，不是方向向量投影

**修复**:
```cpp
// 步骤1: 归一化得到方向向量
double ray_norm = sqrt(x*x + y*y + z*z);
double ray_x = x / ray_norm;
double ray_y = y / ray_norm;
double ray_z = z / ray_norm;

// 步骤2: 计算方向向量在xy平面的投影长度
double norm = sqrt(ray_x*ray_x + ray_y*ray_y);

// 步骤3: 使用方向向量分量进行投影计算
double theta = atan(-ray_z / norm);
double x_norm = ray_x / norm * rho;
double y_norm = ray_y / norm * rho;
```

### 2. 其他已修复问题

- ✅ 3D坐标传递：使用CV_64FC3传递完整坐标
- ✅ 外参转换：camera-to-body → body-to-camera
- ✅ 相机高度：从外参文件提取（0.759-0.762m）
- ✅ 地面平面：z=0（地面坐标系）
- ✅ 段错误：添加空检查

## 📊 调试结果

### 投影计算验证

**测试点**:
```
Input 3D: (-1.03064, -0.231097, 0.757546)
ray_norm: 1.29981
Direction: (-0.792918, -0.177793, 0.582813)
norm (xy projection): 0.812607
theta: -0.622186 rad (-35.6486°)
rho: 450.346
Pixel coords: (522.085, 666.03)
Image size: 1920x1536
```

**分析**:
- ✅ ray_norm计算正确（约1.3）
- ✅ 方向向量归一化正确
- ✅ norm计算正确（xy投影长度）
- ✅ theta计算正确（-35.6°）
- ✅ rho计算正确（多项式结果）
- ✅ 像素坐标在图像范围内（522, 666）

### 输出文件状态

```
-rw-rw-r-- 1 nio nio  11K  before_all_calib.png
-rw-rw-r-- 1 nio nio 4.6K  bev_back.png
-rw-rw-r-- 1 nio nio 5.3K  bev_front.png
-rw-rw-r-- 1 nio nio 9.9K  bev_left.png
-rw-rw-r-- 1 nio nio 4.6K  bev_right.png
```

**问题**: 文件仍然很小，大部分是黑色

## 🔍 可能原因

### 1. KG参数（地面分辨率）不正确

**当前设置**:
```cpp
double ground_resolution = 0.01;  // 1 cm/像素
KG(0, 0) = 1.0 / ground_resolution;
KG(1, 1) = -1.0 / ground_resolution;
```

**问题**: 这个值是猜测的，可能不适合这些相机

**影响**: 
- 如果分辨率太大（0.01），地面范围太小
- 导致投影点都在相机附近，看不到远处

**验证方法**:
```cpp
// 在project_on_ground中添加调试
Mat xy_points = eigen2mat(K_G.inverse()) * p_G;
cout << "Ground point range x: [" << min_x << ", " << max_x << "]" << endl;
cout << "Ground point range y: [" << min_y << ", " << max_y << "]" << endl;
```

### 2. 输入3D点范围不正确

**问题**: ground点的x,y范围可能太小

**检查**:
- BEV尺寸: 1000x1000像素
- KG: 100 pixel/m（0.01 m/pixel）
- Ground范围: 10m x 10m
- 如果相机高度0.76m，FOV约120°，地面范围应该更大

**建议**:
```cpp
// 尝试不同的地面分辨率
double ground_resolution = 0.005;  // 0.5 cm/pixel  → 5m x 5m范围
double ground_resolution = 0.02;   // 2 cm/pixel   → 20m x 20m范围
double ground_resolution = 0.05;   // 5 cm/pixel   → 50m x 50m范围
```

### 3. 投影后坐标范围检查

**问题**: 即使像素坐标在范围内，可能大部分点在图像外

**检查**:
```cpp
// 在distortPointsOcamFull后检查
int in_range = 0, out_range = 0;
for (int i = 0; i < size; i++) {
  double u = p_GC.at<Vec2d>(0, i)[0];
  double v = p_GC.at<Vec2d>(0, i)[1];
  if (u >= 0 && u < img.cols && v >= 0 && v < img.rows)
    in_range++;
  else
    out_range++;
}
cout << in_range << " in range, " << out_range << " out of range" << endl;
```

## 🎯 下一步行动

### 立即行动（30分钟）

1. **调整KG参数**:
   ```cpp
   // 在initializeKG函数中尝试不同值
   // 当前: 0.01 m/pixel
   // 尝试: 0.005, 0.02, 0.05
   ```

2. **添加地面点范围调试**:
   ```cpp
   // 在project_on_ground中添加
   Mat xy_points = eigen2mat(K_G.inverse()) * p_G;
   cout << "Ground x range: [" << min_x << ", " << max_x << "]" << endl;
   cout << "Ground y range: [" << min_y << ", " << max_y << "]" << endl;
   ```

### 短期行动（1-2小时）

3. **创建最小化测试**:
   ```cpp
   // 测试单个点的投影
   // 输入: (0, 0, -0.76) - 相机正下方
   // 期望: (cx, cy) - 主点附近
   ```

4. **检查外参转换**:
   - 验证T_CG矩阵是否正确
   - 验证ground→camera转换

### 中期行动（本周）

5. **运行完整标定**:
   - 修复BEV问题后，运行完整流程
   - 验证Loss值下降
   - 检查优化结果

## 📝 关键发现

### 已确认正确的数学实现

1. **归一化**: 必须将3D点转换为方向向量
   ```cpp
   double ray_norm = sqrt(x*x + y*y + z*z);
   double ray_x = x / ray_norm;
   double ray_y = y / ray_norm;
   double ray_z = z / ray_norm;
   ```

2. **方向向量投影**: 计算在xy平面的投影长度
   ```cpp
   double norm = sqrt(ray_x*ray_x + ray_y*ray_y);
   ```

3. **入射角**: 使用方向向量分量计算
   ```cpp
   double theta = atan(-ray_z / norm);
   ```

4. **投影**: 使用方向向量分量和rho
   ```cpp
   double x_norm = ray_x / norm * rho;
   double y_norm = ray_y / norm * rho;
   ```

### 需要进一步验证的参数

1. **KG参数**: ground_resolution可能需要调整
2. **输入范围**: ground点的x,y范围可能需要扩大
3. **坐标范围**: 投影后的u,v坐标范围需要检查

## 🚀 快速测试建议

### 测试1: 调整KG参数

```bash
# 修改src/optimizer.cpp:468-474
double ground_resolution = 0.005;  // 0.5 cm/pixel
# 重新编译测试
```

### 测试2: 检查ground点范围

```cpp
// 在project_on_ground中添加
double min_x = 1e9, max_x = -1e9, min_y = 1e9, max_y = -1e9;
for (int i = 0; i < rows * cols; i++) {
  min_x = min(min_x, P_G.at<double>(0, i));
  max_x = max(max_x, P_G.at<double>(0, i));
  min_y = min(min_y, P_G.at<double>(1, i));
  max_y = max(max_y, P_G.at<double>(1, i));
}
cout << "Ground X range: [" << min_x << ", " << max_x << "] m" << endl;
cout << "Ground Y range: [" << min_y << ", " << max_y << "] m" << endl;
```

### 测试3: 检查投影后坐标范围

```cpp
// 在distortPointsOcamFull后检查
int in_range = 0, out_range = 0;
double min_u = 1e9, max_u = -1e9, min_v = 1e9, max_v = -1e9;
for (int i = 0; i < P_GC1.cols; i++) {
  double u = p_GC.at<Vec2d>(0, i)[0];
  double v = p_GC.at<Vec2d>(0, i)[1];
  min_u = min(min_u, u); max_u = max(max_u, u);
  min_v = min(min_v, v); max_v = max(max_v, v);
  if (u >= 0 && u < img.cols && v >= 0 && v < img.rows) in_range++;
  else out_range++;
}
cout << "Projection U range: [" << min_u << ", " << max_u << "]" << endl;
cout << "Projection V range: [" << min_v << ", " << max_v << "]" << endl;
cout << "In range: " << in_range << ", Out of range: " << out_range << endl;
```

---

## 💡 建议

**最可能的原因**: KG参数（ground_resolution）不正确

**优先级**:
1. **高**: 尝试不同的ground_resolution值（0.005, 0.02, 0.05）
2. **中**: 添加ground点范围调试输出
3. **中**: 添加投影后坐标范围调试输出
4. **低**: 创建最小化测试程序

**成功标准**:
- BEV图像文件大小显著增加（>100KB）
- 能看到道路、车辆等场景内容
- 交通线在不同相机间可见

---

**最后更新**: 2025-03-27 22:10
**状态**: 投影函数数学已修复，待验证KG参数
**建议**: 立即尝试调整ground_resolution参数
