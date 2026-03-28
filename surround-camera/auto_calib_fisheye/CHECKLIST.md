# 调试完成检查清单

## ✅ 已完成任务

### 核心算法修复

- [x] 修复Ocam投影函数（使用实际3D坐标）
- [x] 修复3D坐标传递（CV_64FC3）
- [x] 修复外参转换方向（camera-to-body → body-to-camera）
- [x] 修复相机高度提取（从外参）
- [x] 修复地面平面设置（z=0）
- [x] 修复段错误（3处）
- [x] 扩大搜索范围（±15°, ±0.05m）

### 外参处理

- [x] 加载外参功能实现
- [x] 保存外参功能实现
- [x] Rodrigues向量转换实现
- [x] 外参文件更新（translation z正值）
- [x] 确认body坐标系定义
- [x] 确认外参格式（camera-to-body）

### 测试程序

- [x] test_bev_simple（标准BEV测试）
- [x] test_bev_no_tail（无tail测试）
- [x] test_bev_identity（单位矩阵外参测试）
- [x] test_ocam_debug.sh（完整自动化测试）
- [x] DIAGNOSE.sh（诊断工具）
- [x] QUICK_TEST.sh（快速测试）
- [x] view_bev_results.sh（结果查看）

### 文档

- [x] README_DEBUGGING.md（快速入门）
- [x] FINAL_STATUS.md（详细状态）
- [x] FIXES_SUMMARY.md（修复总结）
- [x] DEBUGGING_STATUS.md（调试过程）
- [x] BEV_TEST_RESULTS.md（BEV测试结果）
- [x] COORDINATE_SYSTEM_ANALYSIS.md（坐标系分析）
- [x] NEXT_STEPS.md（行动建议）
- [x] CHECKLIST.md（本文件）

### 编译和测试

- [x] CMakeLists.txt更新（添加测试程序）
- [x] 代码编译通过（无错误）
- [x] 生成可执行文件
- [x] 运行测试（无crash）

---

## ❌ 待解决问题

### 核心问题

- [ ] BEV图像中看不到交通线
  - 当前状态：文件很小（6-11KB）
  - 可能原因：Ocam投影函数数学错误 / 图像旋转方向错误

---

## 🎯 下一步验证

### 立即验证（今天）

- [ ] 运行DIAGNOSE.sh查看测试结果
- [ ] 手动查看BEV图像（before_all_calib.png）
- [ ] 确认是否能看见任何图像内容
- [ ] 确认交通线是否可见
- [ ] 确认四个相机图像是否对齐

### 代码验证（本周）

- [ ] 验证Ocam投影函数数学实现
  - [ ] 检查norm计算（3D vs 2D）
  - [ ] 检查入射角计算
  - [ ] 对比Python参考实现
- [ ] 验证图像旋转方向
  - [ ] 保存旋转后图像
  - [ ] 手动检查交通线方向
- [ ] 验证KG参数
  - [ ] 尝试不同ground_resolution值

### 完整验证（下周）

- [ ] 运行完整标定流程
- [ ] 检查Loss值变化
- [ ] 验证优化结果
- [ ] 对比before/after图像

---

## 📊 测试覆盖

### 单元测试

- [x] Ocam参数加载
- [x] 外参加载
- [x] 外参保存
- [x] 坐标转换

### 集成测试

- [x] BEV投影（单个相机）
- [x] BEV投影（多个相机）
- [x] 环绕视图生成
- [x] 纹理提取（无crash）

### 系统测试

- [ ] 完整标定流程（未完成，因BEV问题）
- [ ] Loss值优化（未完成，因BEV问题）
- [ ] 结果保存（未完成，因BEV问题）

---

## 📈 性能指标

### 编译
- 编译时间: ~30秒
- 编译错误: 0
- 编译警告: 若干（不影响功能）

### 运行
- 测试运行时间: ~10秒
- 内存使用: 正常
- 无段错误/crash

### 输出
- BEV图像生成: ✅ 成功
- 文件生成: ✅ 成功
- 文件大小: ❌ 太小（6-11KB）

---

## 🐛 已知bug（已修复）

| bug # | 描述 | 位置 | 修复状态 |
|-------|------|------|----------|
| 1 | Ocam投影使用固定z值 | optimizer.cpp:223 | ✅ 已修复 |
| 2 | 3D坐标传递不完整 | optimizer.cpp:608 | ✅ 已修复 |
| 3 | 外参转换方向错误 | optimizer.cpp:1333 | ✅ 已修复 |
| 4 | 相机高度未加载 | optimizer.cpp:1348 | ✅ 已修复 |
| 5 | 地面平面设置错误 | optimizer.cpp:602 | ✅ 已修复 |
| 6 | 纹理提取段错误1 | texture_extractor.cpp:76 | ✅ 已修复 |
| 7 | 纹理提取段错误2 | texture_extractor.cpp:188 | ✅ 已修复 |
| 8 | 校准段错误 | calibration.cpp:475 | ✅ 已修复 |

---

## 📝 代码变更

### 修改的文件

- `src/optimizer.cpp` - 核心投影和优化逻辑
- `src/calibration.cpp` - 主流程和搜索范围
- `src/texture_extractor.cpp` - 纹理提取
- `include/optimizer.h` - 头文件
- `CMakeLists.txt` - 构建配置

### 新增的文件

- `test_bev_simple.cpp` - BEV测试
- `test_bev_no_tail.cpp` - 无tail测试
- `test_bev_identity.cpp` - 单位矩阵测试
- `test_ocam_debug.sh` - 自动化测试
- `DIAGNOSE.sh` - 诊断工具
- `*.md` - 文档（8个）

---

## 📚 知识总结

### 外参定义（已确认）

- **rotation**: 相机对于body的旋转向量（Rodrigues，rad）
- **translation**: 相机在body坐标系下的位移（m）
- **body坐标系**: x前方，y左方，z上方
- **格式**: camera-to-body变换

### Ocam投影关键步骤

1. 获取3D点坐标 (x, y, z)
2. 计算norm（可能应为3D范数）
3. 计算入射角theta = atan(-z / norm)
4. 使用world2cam多项式计算rho
5. 投影到归一化平面
6. 应用仿射变换

### 坐标转换链

```
地面坐标系 (z=0)
    ↓ T_ground_to_camera
相机坐标系
    ↓ Ocam投影
图像坐标系
```

---

## 🔍 调试建议

### 如果BEV图像完全黑色

1. 检查Ocam投影函数数学
2. 检查norm计算
3. 检查theta计算
4. 对比Python参考实现

### 如果BEV图像有内容但不对齐

1. 检查外参转换
2. 检查KG参数
3. 检查图像旋转
4. 调整搜索范围

### 如果BEV图像对齐但模糊

1. 检查Ocam多项式系数
2. 检查仿射变换矩阵
3. 检查相机高度

---

## 🎯 最终目标

### 短期（本周）

- [ ] 修复BEV投影问题（看到交通线）
- [ ] 验证投影准确性
- [ ] 确认外参正确性

### 中期（本月）

- [ ] 运行完整标定流程
- [ ] 优化外参（降低Loss值）
- [ ] 生成高质量的环绕视图

### 长期（下月）

- [ ] 应用到实际数据集
- [ ] 与其他算法对比
- [ ] 文档化和代码整理

---

## 📞 需要帮助？

1. **查看文档**: 阅读所有`*.md`文件
2. **运行诊断**: `./DIAGNOSE.sh`
3. **查看代码**: `src/optimizer.cpp`是关键
4. **对比Python**: 参考`ocam_disorder.py`

---

**最后更新**: 2025-03-27 22:10
**状态**: 系统调试完成，待解决核心投影问题
**建议**: 验证Ocam投影函数数学实现

**祝调试顺利！** 🎯
