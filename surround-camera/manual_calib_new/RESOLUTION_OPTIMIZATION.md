# 分辨率优化说明

## 优化内容

### 1. BEV 尺寸提升
- **旧值**: 1000 x 1000 像素
- **新值**: 2000 x 2000 像素
- **提升**: 4倍像素数量

### 2. 像素精度提升
- **旧值**: 1.0 cm/pixel (metric_ratio_m = 0.01)
- **新值**: 0.5 cm/pixel (metric_ratio_m = 0.005)
- **提升**: 2倍精度

### 3. 显示窗口提升
- **旧值**: 1000 x 1000
- **新值**: 1200 x 1200
- **提升**: 44%显示面积

### 4. 输出图像
- stitching.png 现在输出 **2000x2000** 高分辨率图像

## 保持不变的参数

- 中心黑色区域: 200x200 (通过比例计算自动适应)
- Tail 大小: 300px
- 融合边界: 40%-60% 比例

## 代码修改

```cpp
// BEV parameters - RESOLUTION OPTIMIZED
int bev_rows = 2000, bev_cols = 2000;
double metric_ratio_m = 0.005;  // 0.5cm/pixel

// Blend region proportional to BEV size
int blend_start = bev_rows * 0.4;  // 800 for 2000px
int blend_end = bev_rows * 0.6;    // 1200 for 2000px

// Window size
int width = 1200, height = 1200;
```

## 使用方法

```bash
./bin/run_avm_new ./imgs ./ocam_params ./extrinsics.json 1
```

输出 stitching.png 为 2000x2000 分辨率。
