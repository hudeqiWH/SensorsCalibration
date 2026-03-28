# Manual Calib 重写总结

## 参考依据

基于 `auto_calib_fisheye/test_bev_simple.cpp` 的实现进行重写。

## 主要适配内容

### 1. 新的内参格式 (Ocam 模型)

**旧格式 (intrinsic_param.hpp):**
```cpp
// 从 JSON 读取: root[id]["param"]["cam_K"]["data"]
// 从 JSON 读取: root[id]["param"]["cam_dist"]["data"]
K << intri[0][0].asDouble(), ...
dist.push_back(d[0][i].asDouble());
```

**新格式 (参考 optimizer.cpp:1299):**
```cpp
// Ocam 参数结构
struct OcamParams {
    double cx, cy;                    // 主点
    double c, d, e;                   // 仿射参数
    vector<double> cam2world_poly;    // 相机到世界多项式
    vector<double> world2cam_poly;    // 世界到相机多项式
    Eigen::VectorXd world2cam_coeffs; // Horner 方法系数
    int width, height;
};

// 从 JSON 读取
params.cx = intrinsic["principal_point"][0].asDouble();
params.cy = intrinsic["principal_point"][1].asDouble();
params.c = intrinsic["affine_c"].asDouble();
...
```

### 2. 新的外参格式

**旧格式 (extrinsic_param.hpp):**
```cpp
// 从 JSON 读取: root[id]["param"]["sensor_calib"]["data"]
// 直接读取 4x4 矩阵
extrinsic << data[0][0].asDouble(), ...
```

**新格式 (参考 optimizer.cpp:1397):**
```cpp
// 从 JSON 读取 rotation 和 translation
vector<double> rvec(3);
for (int i = 0; i < 3; i++) {
    rvec[i] = cam_ext["rotation"][i].asDouble();
}
Eigen::Vector3d translation_vec;
for (int i = 0; i < 3; i++) {
    translation_vec[i] = cam_ext["translation"][i].asDouble();
}

// 转换 Rodrigues 向量到旋转矩阵
Eigen::Matrix3d R_cam_to_body = rodriguesToRotationMatrix(rvec);

// 构建 T_cam_to_body 并求逆得到 T_body_to_cam
Eigen::Matrix4d T_cam_to_body = Eigen::Matrix4d::Identity();
T_cam_to_body.block<3, 3>(0, 0) = R_cam_to_body;
T_cam_to_body.block<3, 1>(0, 3) = translation_vec;
cam.extrinsic = T_cam_to_body.inverse();  // 这就是 T_vehicle_to_camera
```

### 3. 新的 BEV 投影方式 (CPAC 公式)

**旧投影 (run_avm.cpp:107):**
```cpp
// 简单的 K_G * p_G * height 投影
cv::Mat P_G = cv::Mat::ones(4, rows * cols, CV_64FC1);
P_G(cv::Rect(0, 0, rows * cols, 3)) = K_G.inv() * p_G * height;
cv::Mat P_GC = T_CG_ * P_G;
// ... 然后使用 world2cam 或 OpenCV fisheye 投影
```

**新投影 (参考 optimizer.cpp:621):**
```cpp
// CPAC BEV 坐标生成
// Body 坐标系: x=forward, y=left, z=up
for (int j = 0; j < bev_rows; j++) {
    for (int i = 0; i < bev_cols; i++) {
        int idx = j * bev_cols + i;
        double wheel_base_pixel = wheel_base / metric_ratio_m;
        double veh_x = (wheel_base_pixel / 2.0 + bev_rows / 2.0 - j) * metric_ratio_m;
        double veh_y = (bev_cols / 2.0 - i) * metric_ratio_m;
        P_G.at<double>(0, idx) = veh_x;  // x_forward
        P_G.at<double>(1, idx) = veh_y;  // y_left
        P_G.at<double>(2, idx) = 0.0;    // z_up
    }
}

// 变换到相机坐标系
Eigen::MatrixXd P_GC_eigen = cam.extrinsic * P_G_eigen;

// 使用 Ocam 完整模型投影
distortPointsOcamFull(P_GC1, p_GC, cam.ocam);
```

### 4. Ocam 投影实现

**参考 optimizer.cpp:217:**
```cpp
void distortPointsOcamFull(Mat &P_GC1, Mat &p_GC, const OcamParams &ocam) {
    for (int i = 0; i < P_GC1.cols; i++) {
        double x = P_GC1.at<Vec3d>(0, i)[0];
        double y = P_GC1.at<Vec3d>(0, i)[1];
        double z = P_GC1.at<Vec3d>(0, i)[2];
        
        // 检查点是否在相机前方
        if (z < 0) { ... }
        
        // CPAC 公式
        double norm = sqrt(x * x + y * y);
        double theta = atan(-z / norm);
        
        // 使用 Horner 方法计算多项式
        double rho = 0.0;
        for (int k = 0; k < ocam.world2cam_coeffs.size(); k++) {
            rho = rho * theta + ocam.world2cam_coeffs(k);
        }
        
        // 仿射变换
        double uu = x / norm * rho;
        double vv = y / norm * rho;
        double u = uu + vv * ocam.e + ocam.cx;
        double v = uu * ocam.d + vv * ocam.c + ocam.cy;
    }
}
```

## 关键变化总结

| 组件 | 旧版 | 新版 |
|-----|------|------|
| **相机模型** | 简单畸变系数 | Ocam 多项式模型 |
| **内参读取** | cam_K + cam_dist | principal_point + affine + polynomials |
| **外参格式** | 直接 4x4 矩阵 | rotation + translation (Rodrigues) |
| **坐标系** | 自定义 | CPAC 标准 (x=F, y=L, z=U) |
| **BEV 生成** | K_G.inv() * p_G * height | CPAC 公式 |
| **投影方式** | world2cam / fisheye::distortPoints | distortPointsOcamFull |
| **高度来源** | 固定值 | 从外参 translation[2] 计算 |

## 验证结果

编译成功，程序可正常运行：
```
$ ./bin/run_avm_new
Usage: ./run_avm_new <image_path> <ocam_param_path> <extrinsic_json> [camera_model]
  camera_model: 0=fisheye, 1=ocam(default), 2=pinhole
```
