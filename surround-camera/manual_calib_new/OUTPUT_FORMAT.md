# 标定结果输出格式说明

## 输出文件

### 1. 单个相机标定文件
**文件名**: `calibration_0.txt`, `calibration_1.txt`, `calibration_2.txt`, `calibration_3.txt`

**内容格式**:
```
Extrinsic (T_body_to_cam):
R:
[r11] [r12] [r13]
[r21] [r22] [r23]
[r31] [r32] [r33]
t: [tx] [ty] [tz]
height: [camera_height]

************* Output JSON format (T_cam_to_body) *************
"park_front": {
    "rotation": [rx, ry, rz],
    "translation": [tx, ty, tz]
}
```

### 2. 完整外参 JSON 文件
**文件名**: `camera_extrinsics_calibrated.json`

**格式与输入完全一致**:
```json
{
    "extrinsic_param": {
        "park_front": {
            "rotation": [rx, ry, rz],
            "translation": [tx, ty, tz]
        },
        "park_left": {
            "rotation": [rx, ry, rz],
            "translation": [tx, ty, tz]
        },
        "park_back": {
            "rotation": [rx, ry, rz],
            "translation": [tx, ty, tz]
        },
        "park_right": {
            "rotation": [rx, ry, rz],
            "translation": [tx, ty, tz]
        }
    }
}
```

## 坐标变换说明

### 程序内部使用 (T_body_to_cam)
- 表示从车身坐标系到相机坐标系的变换
- 用于 BEV 投影计算

### 输出格式 (T_cam_to_body)
- 表示从相机坐标系到车身坐标系的变换
- 与输入格式一致
- `rotation`: Rodrigues 旋转向量 (3x1)
- `translation`: 相机在车身坐标系中的位置 (3x1)

### 变换关系
```
T_cam_to_body = inverse(T_body_to_cam)
```

## 使用方式

### 直接使用标定结果
```bash
# 标定完成后，camera_extrinsics_calibrated.json 可直接作为其他程序的输入
./other_program --extrinsics camera_extrinsics_calibrated.json
```

### 加载单个相机参数
```cpp
// 从 calibration_X.txt 中读取 JSON 格式部分
Json::Value root;
Json::Value cam = root["park_front"];
std::vector<double> rvec = cam["rotation"];
std::vector<double> tvec = cam["translation"];
```

## 相机编号对应

| Frame ID | 相机名称 | JSON Key |
|---------|---------|----------|
| 0 | Front | park_front |
| 1 | Left | park_left |
| 2 | Back | park_back |
| 3 | Right | park_right |
