# 坐标系定义分析

## 用户关键信息

**"传入的相机外参是相机相对于机器人本体中心的"**

这意味着：外参描述的是**相机在机器人本体坐标系中的位姿**。

## 坐标系定义

### 机器人本体坐标系（Body Frame）
- 原点：机器人中心
- X轴：向前（行驶方向）
- Y轴：向左
- Z轴：向上

### 相机坐标系（Camera Frame）
- 原点：相机光心
- X轴：图像右方向
- Y轴：图像下方向
- Z轴：光轴方向（向前/镜头朝向）

### 地面坐标系（Ground Frame）
- 原点：地面上某点
- X轴：前向
- Y轴：左向
- Z轴：向上

## 外参的数学含义

### 情况1: 外参 = camera-to-body（T_cam_to_body）

如果外参表示从**相机坐标系**到**本体坐标系**的变换：
```
P_body = R * P_cam + t
```
其中：
- R, t = camera-to-body变换
- P_cam: 点在相机坐标系中的坐标
- P_body: 点在本体坐标系中的坐标

**物理意义**：t是相机原点在本体坐标系中的坐标（相机位置）

**投影公式**：
```
P_cam = R^T * (P_body - t)
```

### 情况2: 外参 = body-to-camera（T_body_to_cam）

如果外参表示从**本体坐标系**到**相机坐标系**的变换：
```
P_cam = R * P_body + t
```

**物理意义**：t是本体原点在相机坐标系中的坐标

**投影公式**：
```
P_cam = R * P_body + t
```

## 根据用户信息的推理

用户说："相机相对于机器人本体中心"

这通常意味着：**已知相机在本体坐标系中的位置和姿态**

也就是：**T_body_to_camera = [R | t]**，其中：
- R: 从本体到相机的旋转
- t: 相机光心在本体坐标系中的坐标

但通常存储的是**camera-to-body**，因为：
- 更容易测量（相机姿态相对于本体）
- 更常用

### 当前代码的理解

代码中的注释：
```cpp
// The translation in file is camera position in body frame: t_cam_in_body
// For projection, we need: T = [R_body_to_cam | -R_body_to_cam * t_cam_in_body]
```

假设外参文件包含：**camera-to-body**变换
- rotation_cam_to_body: R_cb
- translation_vec: t_cam_in_body（相机在本体中的位置）

那么body-to-camera变换为：
- R_bc = R_cb^T
- t_bc = -R_bc * t_cam_in_body = -R_cb^T * t_cam_in_body

这与我当前的代码一致。

### 物理合理性检查

当前代码提取的相机高度：
- front: 0.160m
- left: 0.160m
- back: 0.160m
- right: 0.163m

**问题**：外参文件中的translation z值：
- front: -0.255
- left: -0.137
- back: -0.137
- right: -0.211

如果translation_vec是相机在body坐标系中的位置，那么z应该是**正值**（相机在地面之上）。但文件中是**负值**。

**可能的解释**：
1. 文件中存储的是body-to-camera而非camera-to-body
2. 或者z轴方向定义不同（向下为正）
3. 或者translation需要符号反转

### 假设：文件中存储的是body-to-camera

如果文件存储的是**body-to-camera**：
- R_bc = rotation_cam_to_body (直接使用)
- t_bc = translation_vec (直接使用)

那么投影应该是：
```cpp
// 不需要转置和负号
Eigen::Matrix3d rotation = rotation_cam_to_body;  // 直接使用
Eigen::Vector3d translation = translation_vec;     // 直接使用
```

### 验证方法

**方法1**: 检查translation的z值符号
- 如果相机在地面之上，z应为正值
- 如果文件中z为负，可能需要反转符号

**方法2**: 创建简单测试
```cpp
// 测试1：使用当前代码（camera-to-body假设）
// 测试2：使用body-to-camera假设（不转置）
// 比较哪个能看到内容
```

**方法3**: 检查fisheye实现
- 如果fisheye使用不同的转换，对比差异

## 📊 当前代码分析

### 外参加载代码
```cpp
// Convert Rodrigues vector to rotation matrix (camera-to-body)
Eigen::Matrix3d rotation_cam_to_body = rodriguesToRotationMatrix(rvec);

// We need ground-to-camera transformation for projecting ground points to camera
// The extrinsics file contains camera-to-body, but we need body-to-camera for projection
// The translation in file is camera position in body frame: t_cam_in_body
// For projection, we need: T = [R_body_to_cam | -R_body_to_cam * t_cam_in_body]
Eigen::Matrix3d rotation = rotation_cam_to_body.transpose();
Eigen::Vector3d translation = -rotation * translation_vec;  // -R^T * t_cam_in_body
```

**假设**: camera-to-body存储在文件中
**转换**: body-to-camera = [R^T | -R^T * t]

### BEV投影代码
```cpp
Mat P_G = Mat::ones(4, rows * cols, CV_64FC1);
Mat xy_points = eigen2mat(K_G.inverse()) * p_G;
P_G(Rect(0, 0, rows * cols, 2)) = xy_points(Rect(0, 0, rows * cols, 2));
P_G(Rect(0, 2, rows * cols, 1)) = cv::Mat::zeros(rows * cols, 1, CV_64FC1);
P_GC = T_CG_ * P_G;
```

**坐标系**: 
- P_G: 地面点在ground坐标系中（z=0）
- T_CG: ground-to-camera变换
- P_GC: 地面点在camera坐标系中

## 🤔 关键问题

### 问题1: T_CG 是 ground-to-camera 还是 camera-to-ground？

函数名`project_on_ground`暗示：
- 输入：图像
- 输出：地面投影

但参数是`T_CG`，注释说"ground->camera"。

如果T_CG是ground-to-camera，那么：
```
P_cam = T_CG * P_ground
```

然后从P_cam投影到图像。

这逻辑上合理。

### 问题2: Ground坐标系与Body坐标系的关系

假设：
- Ground坐标系 = Body坐标系（平移差别忽略）
- 或者T_ground_body是已知的

如果地面在原点，body在(0,0,h)，那么：
- P_ground = [I | -h] * P_body

但T_CG应该包含这个高度信息。

### 问题3: height参数的使用

代码中：
```cpp
opt.project_on_ground(imgf, opt.extrinsic_front, ..., opt.hf, "front")
```

其中opt.hf是相机高度（0.16m）。

但外参的translation已经包含了高度信息（z=-0.255）。

**矛盾**: 
- 外参translation[2] = -0.255（相机在地面下）
- opt.hf = 0.160（相机高度）

这可能意味着：
1. translation_vec不需要用于投影，仅用于提取高度
2. 或者height和外参translation有冗余/冲突

## 💡 假设与验证

### 假设1: 外参文件存储camera-to-body，translation_z符号错误

**依据**:
- 代码假设camera-to-body
- 但translation_z为负（物理不合理）

**验证**:
```cpp
// 反转translation符号
double camera_height = fabs(translation_vec[2]);  // 使用绝对值
// 或者
Eigen::Vector3d translation_vec_corrected = -translation_vec;
Eigen::Vector3d translation = -rotation * translation_vec_corrected;
```

### 假设2: 外参文件存储body-to-camera

**依据**:
- "相机相对于机器人本体中心"可能意味着body-to-camera
- translation直接使用，不需要负号

**验证**:
```cpp
// 不使用转置和负号
Eigen::Matrix3d rotation = rotation_cam_to_body;  // 直接使用
Eigen::Vector3d translation = translation_vec;     // 直接使用
```

### 假设3: height参数不应传递给project_on_ground

**依据**:
- 外参已经包含了相机高度信息
- height参数可能是多余的

**验证**:
```cpp
// 不使用height参数
Mat GF = opt.project_on_ground(imgf, opt.extrinsic_front, opt.intrinsic_front,
                               opt.distortion_params_front, opt.KG, opt.brows,
                               opt.bcols, /*hf*/1.0, "front");
```

### 假设4: KG定义错误

**依据**:
- KG应该直接将像素转换为物理坐标
- 但代码中ground_resolution = 0.01是猜测值

**验证**:
- 计算实际所需的KG值
- 或从Ocam参数推导

## 🎯 建议的下一步

### 高优先级

1. **验证外参文件定义** (30分钟)
   - 确认旋转向量格式
   - 确认translation物理意义
   - 确认坐标系定义

2. **测试translation符号反转** (30分钟)
   ```cpp
   // 在loadExtrinsicsFromJson中尝试
   Eigen::Vector3d translation_vec_corrected = -translation_vec;
   Eigen::Vector3d translation = -rotation * translation_vec_corrected;
   ```

3. **测试body-to-camera假设** (30分钟)
   ```cpp
   // 不使用转置
   Eigen::Matrix3d rotation = rotation_cam_to_body;
   Eigen::Vector3d translation = translation_vec;
   ```

### 中优先级

4. **保存旋转后的图像** (20分钟)
   ```cpp
   imwrite("debug_rotated_front.png", imgf);
   // ...
   ```

5. **对比fisheye实现** (1小时)
   - 检查fisheye模式是否工作
   - 对比坐标转换差异

### 低优先级

6. **推导正确的KG** (2小时)
   - 从Ocam参数计算
   - 验证ground_resolution

## 🔍 物理直觉

如果相机安装在车辆上，朝向下方的地面：
- 相机高度 ~ 0.16m
- 地面在相机下方
- 在相机坐标系中，地面点z坐标为正值（在相机前方）

外参translation应该表示相机在body坐标系中的位置：
- 如果body坐标系z向上
- 相机在地面之上，translation_z应为正

但文件中translation_z为负，这表明：
1. 文件中存储的是camera-to-body
2. 或者z轴方向定义不同（向下为正）
3. 或者需要符号反转

**最简单的修复**: 反转translation符号
```cpp
translation_vec = -translation_vec;  // 反转所有分量
// 或者仅反转z
translation_vec[2] = -translation_vec[2];
```

---

**结论**: 核心问题可能是外参的translation符号错误或外参转换方向错误。建议优先验证这两点。
