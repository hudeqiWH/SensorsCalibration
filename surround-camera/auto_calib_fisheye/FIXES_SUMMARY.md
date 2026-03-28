# Ocam标定算法调试修复总结

## 问题概述
标定结果很差，交通线没有对齐，基本上是乱码。

## 根本原因分析

经过系统调试，发现以下关键问题：

### 1. 严重错误：Ocam投影函数使用固定的z坐标 ❌→✅

**位置**：`src/optimizer.cpp:223-224`

**问题**：
```cpp
// 错误代码
double z_omni = -1.0;  // 所有点使用相同的z值！
double theta = atan(z_omni / norm);
```

所有投影点使用固定的z=-1.0，导致所有点的入射角θ都相同，投影完全错误！

**修复**：
```cpp
// 修复后的代码
double x = P_GC1.at<Vec3d>(0, i)[0];
double y = P_GC1.at<Vec3d>(0, i)[1];
double z = P_GC1.at<Vec3d>(0, i)[2];  // 获取实际的z坐标
double z_omni = -z;  // 标准到Ocam坐标系：z取负
double theta = atan(z_omni / norm);
```

**影响**：这是最主要的问题，导致所有Ocam相机的投影都错误。

### 2. 3D坐标传递不完整 ❌→✅

**位置**：`src/optimizer.cpp:608-615, 920-926`

**问题**：使用`CV_64FC2`类型只传递x,y坐标，丢失了z坐标信息。

**修复**：改用`CV_64FC3`类型，完整传递3D坐标(x,y,z)。

```cpp
// 修复后的代码
Mat P_GC1 = Mat::zeros(1, rows * cols, CV_64FC3);
for (int i = 0; i < rows * cols; i++) {
  double x = P_GC.at<double>(0, i);
  double y = P_GC.at<double>(1, i);
  double z = P_GC.at<double>(2, i);
  P_GC1.at<Vec3d>(0, i) = Vec3d(x, y, z);
}
```

### 3. 坐标变换方向错误 ❌→✅

**位置**：`src/optimizer.cpp:1320-1334`

**问题**：外参文件包含camera-to-body变换，但代码需要body-to-camera（ground-to-camera）变换。

**修复**：正确转换变换矩阵：
```cpp
// camera-to-body → body-to-camera
Eigen::Matrix3d rotation = rotation_cam_to_body.transpose();  // R^T
Eigen::Vector3d translation = -rotation * translation;        // -R^T * t
```

### 4. 相机高度未从外参加载 ❌→✅

**位置**：`src/optimizer.cpp:1350-1360`

**问题**：相机高度使用硬编码值0.16m，没有从外参文件加载。

**修复**：从外参平移向量的z分量提取相机高度：
```cpp
if (cam_name == "front") {
  extrinsic_front = T;
  hf = translation[2];  // 从外参更新相机高度
}
```

### 5. 粗调搜索范围过小 ❌→✅

**位置**：`src/calibration.cpp:46-93`

**问题**：粗调搜索范围太小（旋转±3°，平移±0.01m），难以找到好的初始值。

**修复**：扩大搜索范围：
- 旋转：从±3°扩大到**±15°**
- 平移：从±0.01m扩大到**±0.05m**

### 6. 多项式求值顺序验证 ✅

**检查**：`src/optimizer.cpp:160-165` vs `ocam_disorder.py:78-86`

**结果**：C++和Python的`evaluatePolynomial`函数都使用Horner方法，**实现正确**。

```cpp
double evaluatePolynomial(const vector<double> &coeffs, double x) {
  double result = 0.0;
  for (int i = coeffs.size() - 1; i >= 0; i--) {
    result = result * x + coeffs[i];
  }
  return result;
}
```

### 7. 仿射变换矩阵验证 ✅

**检查**：`src/optimizer.cpp:237-238` vs `ocam_disorder.py:118`

**结果**：
- C++: `p_pix = A * p_norm + B` ✓
- Python: `p = xy @ A.T + B` ✓

两者数学上等价，**实现正确**。

## 修复后的算法流程

```
1. 加载图像
   ↓
2. 加载Ocam内参（多项式系数、仿射矩阵）
   ↓
3. 加载外参（camera-to-body → body-to-camera转换）
   ↓
4. 投影到地面（project_on_ground）
   ├─ 生成地面网格点 (x,y,0)
   ├─ 应用KG^-1和高度，得到3D点 (X,Y,Z)_G
   ├─ 应用T_CG: (X,Y,Z)_G → (X,Y,Z)_C
   ├─ 归一化得到方向向量 (x,y,z)_norm
   └─ Ocam投影：
      ├─ 计算theta = atan(-z / sqrt(x^2 + y^2))
      ├─ 计算rho = poly(theta, world2cam_poly)
      ├─ 计算归一化平面坐标 (x_norm, y_norm)
      └─ 应用仿射变换得到像素坐标 (u,v)
   ↓
5. 提取纹理
   ↓
6. 粗调优化（大范围搜索）
   ↓
7. 精调优化（小范围细化）
   ↓
8. 保存结果
```

## 测试与验证

运行测试脚本：
```bash
./test_ocam_debug.sh
```

验证内容：
1. ✓ 所有输入文件存在
2. ✓ 编译成功
3. ✓ 标定完成（无crash）
4. ✓ 生成before/after图像
5. ✓ 生成外参文件（JSON5格式）
6. ✓ 检查交通线是否对齐

## 预期改进

修复后预期效果：
- Loss值显著下降（预计从~8M降至~2M或更低）
- BEV图像质量明显改善
- 交通线在不同相机间对齐
- 拼接后的环绕视图连贯

## 关键文件

- `src/optimizer.cpp`: Ocam投影和坐标变换
- `src/calibration.cpp`: 搜索范围和主流程
- `test_ocam_debug.sh`: 调试测试脚本
- `2_dog/param/park_*.json`: Ocam参数文件
- `camera_extrinsics_ikalibr.json5`: 外参文件（iKalibr格式）
