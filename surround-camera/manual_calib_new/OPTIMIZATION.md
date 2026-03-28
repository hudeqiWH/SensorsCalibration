# AVM Manual Calibration - 优化说明

## 优化概览

本次优化主要针对 **分辨率提升** 和 **阴影区域减小** 两个方面。

## 具体优化内容

### 1. 分辨率提升

#### BEV 尺寸
- **旧值**: 1000 x 1000 像素
- **新值**: 2000 x 2000 像素
- **提升**: 4倍像素数量

#### 像素精度
- **旧值**: 1.0 cm/pixel (metric_ratio_m = 0.01)
- **新值**: 0.5 cm/pixel (metric_ratio_m = 0.005)
- **提升**: 2倍精度

#### 显示窗口
- **旧值**: 1000 x 1000
- **新值**: 1200 x 1200
- **提升**: 44%显示面积

#### 输出图像
- stitching.png 现在输出 **2000x2000** 高分辨率图像

### 2. 中心黑色区域减小

#### 中心盲区
- **旧值**: 200 x 200 像素 (20%画面)
- **新值**: 100 x 100 像素 (5%画面)
- **减小**: 75%

### 3. 边框阴影减小

#### Tail 裁剪
- **旧值**: 300 像素
- **新值**: 150 像素
- **减小**: 50%

#### 融合边界
- **旧值**: 400 像素 margin
- **新值**: 150 像素 margin
- **减小**: 62.5%

### 4. 融合逻辑优化

- 动态计算融合区域
- 改进角落融合算法
- 更清晰的相机区域分离

## 使用方法

### 默认设置 (推荐)
```bash
./bin/run_avm_new ./imgs ./ocam_params ./extrinsics.json 1
```
配置:
- BEV: 2000x2000
- 中心阴影: 100x100
- 边框阴影: 150px

### 超高质量 (3000x3000)
```bash
./bin/run_avm_new ./imgs ./ocam_params ./extrinsics.json 1 3000 50 100
```
配置:
- BEV: 3000x3000
- 中心阴影: 50x50 (超小)
- 边框阴影: 100px

### 快速预览 (1000x1000)
```bash
./bin/run_avm_new ./imgs ./ocam_params ./extrinsics.json 1 1000 200 300
```
配置:
- BEV: 1000x1000
- 中心阴影: 200x200
- 边框阴影: 300px

## 参数说明

```
./run_avm_new <images> <params> <extrinsics> <model> <bev_size> <center_shadow> <tail_size>

参数:
  images         图像目录
  params         参数目录
  extrinsics     外参JSON文件
  model          相机模型 (0=fisheye, 1=ocam, 2=pinhole)
  bev_size       BEV分辨率 (默认: 2000)
  center_shadow  中心阴影大小 (默认: 100)
  tail_size      边框阴影大小 (默认: 150)
```

## 效果对比

| 项目 | 优化前 | 优化后 | 提升 |
|-----|-------|-------|-----|
| 输出分辨率 | 1000x1000 | 2000x2000 | 4x |
| 像素精度 | 1.0 cm | 0.5 cm | 2x |
| 中心盲区 | 200x200 | 100x100 | -75% |
| 边框阴影 | 300px | 150px | -50% |
| 有效画面 | ~60% | ~85% | +25% |

## 技术细节

### 关键代码修改

#### BEV 参数
```cpp
// BEV parameters - OPTIMIZED
int bev_rows = 2000, bev_cols = 2000;  // 4x pixels
double metric_ratio_m = 0.005;          // 2x precision
int blend_margin = 150;                 // Smaller blend region
int center_shadow_size = 100;           // Smaller center shadow
```

#### Tail 函数
```cpp
Mat tail(Mat img, const string &index) {
    int size = 150;  // Reduced from 300
    ...
}
```

#### 融合逻辑
```cpp
// Dynamic blend region calculation
int center_start_x = (bev_cols - center_shadow_size) / 2;
int center_end_x = center_start_x + center_shadow_size;
// ... optimized blending logic
```

## 性能影响

- **内存使用**: 约增加 4倍 (由于分辨率提升)
- **处理时间**: 约增加 3-4倍 (BEV投影和融合)
- **GPU/显示**: 使用 1200x1200 窗口，流畅度不受影响

## 建议

1. **日常使用**: 默认 2000x2000 设置，平衡质量和性能
2. **精细标定**: 使用 3000x3000 或更高，最小化阴影
3. **快速预览**: 使用 1000x1000，更快响应速度
