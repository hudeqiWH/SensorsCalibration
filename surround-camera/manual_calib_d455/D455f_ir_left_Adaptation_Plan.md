# D455f ir_left 接入标定工具适配方案

> 本文档说明将 FrontLeft 相机从原有鱼眼相机替换为 Intel RealSense D455f `ir_left` 红外相机所需的完整代码改动。适用于组内技术交接。

---

## 一、核心差异概览

| 项目 | 原有 FrontLeft (鱼眼) | D455f ir_left (红外) |
|------|----------------------|----------------------|
| **CameraType** | `FISHEYE_OPENCV = 0` | 需新增 `PINHOLE = 2` |
| **畸变模型** | OpenCV Fisheye 4参数 (k1~k4) | Brown-Conrady (但系数全为0) |
| **投影公式** | `θ = atan(r)` → 多项式畸变 → remap | 标准针孔: `u = fx·(xc/zc) + cx` |
| **图像尺寸** | 3848×2168 (彩色RGB) | 848×480 (灰度/IR) |
| **内参JSON结构** | `{"intrinsic_param": {"camera_matrix": [...], "distortion_coefficients": [...]}}` | `{"intrinsics": {"ir_left": {"fx": ..., "fy": ..., "cx": ..., "cy": ...}}}` |
| **实际畸变** | 有显著畸变 | **零畸变** (coeffs 全0) |

> ⚠️ **重要**：不能简单伪造 Fisheye JSON 并把 k1~k4 填0。OpenCV Fisheye 公式中 `θ = atan(r)` 是非线性的，即使 k1~k4=0，`scale = atan(r)/r ≠ 1`，与真实针孔投影数学不等价。必须新增针孔投影路径。

---

## 二、代码改动清单（共6处）

### 改动1：新增 `PINHOLE` 相机类型枚举

**文件**：`src/run_front3_calib_v4.cpp`
**位置**：第31行

```cpp
// 修改前
enum CameraType { FISHEYE_OPENCV = 0, OCAM = 1 };

// 修改后
enum CameraType { FISHEYE_OPENCV = 0, OCAM = 1, PINHOLE = 2 };
```

---

### 改动2：修改 FL 的相机类型声明

**文件**：`src/run_front3_calib_v4.cpp`
**位置**：第71行

```cpp
// 修改前
const CameraType camera_types[3] = {FISHEYE_OPENCV, OCAM, FISHEYE_OPENCV};

// 修改后
const CameraType camera_types[3] = {PINHOLE, OCAM, FISHEYE_OPENCV};
```

---

### 改动3：新增 D455f ir_left 内参加载函数

**文件**：`src/run_front3_calib_v4.cpp`
**位置**：在 `loadFisheyeIntrinsic()` 之后新增（约第136行之后）

```cpp
/**
 * 加载 D455f ir_left 内参
 * 解析 d455f_calibration_params.json 中的 intrinsics.ir_left 字段
 */
void loadD455IrLeftIntrinsic(FrontCameraConfig &cam, const string &filename) {
    ifstream file(filename);
    if (!file.is_open()) { cerr << "Cannot open: " << filename << endl; exit(1); }
    Json::Value root; Json::Reader reader;
    if (!reader.parse(file, root)) { cerr << "Failed to parse: " << filename << endl; exit(1); }
    
    const Json::Value &ir = root["intrinsics"]["ir_left"];
    cam.width  = ir["width"].asInt();
    cam.height = ir["height"].asInt();
    
    double fx = ir["fx"].asDouble();
    double fy = ir["fy"].asDouble();
    double cx = ir["cx"].asDouble();
    double cy = ir["cy"].asDouble();
    
    cam.camera_matrix = Mat::eye(3, 3, CV_64FC1);
    cam.camera_matrix.at<double>(0, 0) = fx;
    cam.camera_matrix.at<double>(1, 1) = fy;
    cam.camera_matrix.at<double>(0, 2) = cx;
    cam.camera_matrix.at<double>(1, 2) = cy;
    
    // ir_left 畸变系数全为0，dist_coeffs 置空即可
    cam.dist_coeffs = Mat::zeros(1, 5, CV_64FC1);
    
    cout << "Loaded D455 ir_left " << cam.name << ": " << cam.width << "x" << cam.height
         << " fx=" << fx << " fy=" << fy << endl;
}
```

---

### 改动4：`ParallelProjector` 中新增针孔投影分支

**文件**：`src/run_front3_calib_v4.cpp`
**位置**：第238行附近（`operator()` 的 `if (cam_.type == ...)` 链中）

```cpp
// 修改前（第238~262行）
if (cam_.type == FISHEYE_OPENCV) {
    double xc = wall_pt_cam(0);
    double yc = wall_pt_cam(1);
    double zc = wall_pt_cam(2);
    
    if (zc > 0) {
        double xn = xc / zc;
        double yn = yc / zc;
        double r = sqrt(xn * xn + yn * yn);
        
        if (r > 1e-10) {
            double theta = atan(r);
            double theta2 = theta * theta;
            double theta4 = theta2 * theta2;
            double theta6 = theta4 * theta2;
            double theta8 = theta6 * theta2;
            double theta_d = theta * (1 + k1_ * theta2 + k2_ * theta4 + k3_ * theta6 + k4_ * theta8);
            
            double scale = theta_d / r;
            src_u = fx_ * xn * scale + cx_;
            src_v = fy_ * yn * scale + cy_;
            valid = true;
        }
    }
} else {
    // ... OCAM 代码
}

// 修改后
if (cam_.type == FISHEYE_OPENCV) {
    // ↑ 原有 fisheye 代码保持不变 ↑
} else if (cam_.type == PINHOLE) {
    // ---- 新增针孔投影分支 ----
    double xc = wall_pt_cam(0);
    double yc = wall_pt_cam(1);
    double zc = wall_pt_cam(2);
    
    if (zc > 0) {
        double xn = xc / zc;
        double yn = yc / zc;
        src_u = fx_ * xn + cx_;
        src_v = fy_ * yn + cy_;
        valid = true;
    }
    // --------------------------
} else {
    // ↓ 原有 OCAM 代码保持不变 ↓
}
```

> 说明：`fx_, fy_, cx_, cy_` 复用构造函数中预提取的参数变量即可（它们同样对应 `camera_matrix` 的元素），无需新增成员变量。

---

### 改动5：主函数中修改 FL 内参加载入口

**文件**：`src/run_front3_calib_v4.cpp`
**位置**：第603行附近

```cpp
// 修改前
loadFisheyeIntrinsic(cameras[0], intrinsic_path + "/front_left.json");
loadOcamIntrinsic(cameras[1], intrinsic_path + "/park_front.json");
loadFisheyeIntrinsic(cameras[2], intrinsic_path + "/front_right.json");

// 修改后
loadD455IrLeftIntrinsic(cameras[0], intrinsic_path + "/d455f_calibration_params.json");
loadOcamIntrinsic(cameras[1], intrinsic_path + "/park_front.json");
loadFisheyeIntrinsic(cameras[2], intrinsic_path + "/front_right.json");
```

---

### 改动6：灰度图像转 BGR 通道处理

**文件**：`src/run_front3_calib_v4.cpp`
**位置**：第593行附近（图像加载循环中）

```cpp
for (int i = 0; i < 3; i++) {
    cameras[i].img_original = imread(cameras[i].image_file);
    if (cameras[i].img_original.empty()) {
        cerr << "Failed to load: " << cameras[i].image_file << endl;
        return 1;
    }
    // ---- 新增：单通道红外图像转3通道 ----
    if (cameras[i].img_original.channels() == 1) {
        cvtColor(cameras[i].img_original, cameras[i].img_original, COLOR_GRAY2BGR);
    }
    // ------------------------------------
    cout << "  " << cameras[i].name << ": " << cameras[i].img_original.cols << "x" << cameras[i].img_original.rows << endl;
}
```

> 说明：`cameras[1]` 和 `cameras[2]` 本来就是3通道，此条件判断对它们无影响，仅 `cameras[0]`（ir_left）会被转换。

---

## 三、输入文件组织方式

命令行调用格式不变：

```bash
./bin/run_front3_calib_v4 <image_dir> <intrinsic_dir> <extrinsics.json> [wall_distance_m]
```

**`<image_dir>` 下需包含：**

| 文件名 | 来源 | 说明 |
|--------|------|------|
| `front_left.png` | D455f ir_left | 848×480 灰度/红外图像，程序自动转BGR处理 |
| `front.png` | 前视Ocam | 原有图像 |
| `front_right.png` | 右前鱼眼 | 原有图像 |

**`<intrinsic_dir>` 下需包含：**

| 文件名 | 来源 | 说明 |
|--------|------|------|
| `d455f_calibration_params.json` | D455f SDK 输出 | 内含 `intrinsics.ir_left` 字段，程序从中提取 fx/fy/cx/cy |
| `park_front.json` | Ocam 标定 | 原有内参文件 |
| `front_right.json` | 鱼眼标定 | 原有内参文件 |

**`<extrinsics.json>`：格式不变**，仍需包含 `front_left`（现在对应 ir_left 在车体坐标系中的位姿）、`park_front`、`front_right` 的 Rodrigues 旋转向量和平移量。

> ⚠️ **注意区分两种外参**：
> - `d455f_calibration_params.json` 中的 `extrinsics.ir_left_to_rgb`：这是**D455f 内部 sensor-to-sensor 标定**（ir_left 到 RGB camera 的变换），不是车体到相机的外参。**不要用它来替换 `extrinsics.json`**。
> - `<extrinsics.json>` 中的 `front_left`：车体坐标系到 ir_left 的 6-DOF 外参，需要重新标定或估算初始值。

---

## 四、风险点与验证建议

| 关注点 | 风险描述 | 建议措施 |
|--------|---------|---------|
| **分辨率低** | ir_left 848×480 远低于原鱼眼 3848×2168，投影到墙面后有效像素区域很小且模糊 | 确认 wall_distance 设置合理（建议先用单视图 `VIEW_FL` 模式观察投影范围）；必要时降低输出分辨率 `output_width/output_height` |
| **FOV 差异** | ir_left H-FOV 约 87°，远小于鱼眼 ~180°，若外参初始值仍按鱼眼角度设置，可能看不到墙面 | 必须重新提供 ir_left 的合理初始外参，或先在单视图模式下验证 |
| **跨光谱拼接** | ir_left 是红外图，与 front (RGB)、front_right (RGB) 光度不一致 | `front3_calibration_metrics.txt` 中的 **Photometric Loss** 天然会偏高，应以 **Edge Alignment** 和 **Feature Matching** 为主要标定参考指标 |
| **颜色异常** | 灰度转BGR后三通道值相同，与彩色相机融合时拼接缝处会出现灰度带 | 正常现象，不影响标定精度；如对可视化有要求，可考虑伪彩色映射 |
| **无畸变假设** | 当前方案假设 ir_left 完全无畸变（coeffs 全0） | 如后续发现边缘有轻微畸变，可将 `distortion_coeffs` 从 JSON 中读取并加入针孔投影的畸变修正项 `r²·k1 + r⁴·k2 + ...` |

---

## 五、最小化改动替代方案（源码不改时使用）

如果希望**完全不修改 C++ 源码**，需要在数据预处理阶段完成以下工作：

1. 写一个 Python/Shell 脚本读取 `d455f_calibration_params.json`，提取 `ir_left` 的 `fx/fy/cx/cy`。
2. 按照原有 `front_left.json` 的格式**伪造一个鱼眼内参文件**，k1~k4 填入极小值（如 `1e-10`），使 `atan(r)` 的泰勒展开近似为线性。
3. **但**：由于需要把点云/墙点投影到 ir_left 像平面，而投影公式在源码中硬编码，预处理阶段无法介入。

**结论：** 上述替代方案在数学上存在系统性偏差，不推荐。建议按本文档“代码改动清单”进行源码修改。

---

## 六、改动行数统计

| 改动项 | 行数 | 复杂度 |
|--------|------|--------|
| 新增 `PINHOLE` 枚举 | 1行 | 低 |
| 修改 `camera_types[]` | 1行 | 低 |
| 新增 `loadD455IrLeftIntrinsic()` | ~25行 | 中 |
| 新增针孔投影分支 | ~10行 | 中 |
| 修改主函数加载入口 | 1行 | 低 |
| 灰度转BGR | 3行 | 低 |
| **合计** | **~40行** | **低** |

---

## 七、编译与运行

修改后重新编译：

```bash
cd build && make -j4
```

运行示例：

```bash
./bin/run_front3_calib_v4 ./1_h1_front/imgs ./1_h1_front/param ./1_h1_front/param/extrinsics.json 1.8
```

运行时建议步骤：
1. 先切换到 `VIEW_FL` 单视图模式，确认 ir_left 投影区域可见。
2. 如果投影区域太小/太大，调整 `wall_distance`（第4个命令行参数或 GUI 中的 `wall(m)` 滑块）。
3. 正常执行 6-DOF 外参微调，保存结果。

---

*文档版本：v1.0*
*编写日期：2026-06-22*
*适配目标版本：run_front3_calib_v4*
