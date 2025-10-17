# Zhuo-RM

ROS2机器人视觉与导航项目 - 基于Berxel相机、YOLOv8检测与深度SLAM导航

## 🎉 最新更新

### 2025-10-17: SLAM导航集成完成 ✨
**✅ 深度SLAM障碍物检测与导航决策系统上线！**
- ✅ 实时障碍物检测（348 FPS处理速度）
- ✅ 可导航区域分析
- ✅ 智能导航决策（forward/left/right/stop）
- ✅ 双窗口可视化界面
- ✅ ROS2集成准备就绪
- 详见：[SLAM_INTEGRATION.md](SLAM_INTEGRATION.md)

### 2025-10-15: 深度图闪烁修复 🔧
- ✅ EMA时间平滑算法
- ✅ 深度可视化持久化
- ✅ 检测深度采样优化

### 2025-10-11: AMD 780M 性能优化 🚀
- ✅ FPS 从 13 提升到 45-80（3.5-6倍提升）
- ✅ ONNX Runtime GPU 加速
- 详见：[QUICKSTART_AMD780M.md](QUICKSTART_AMD780M.md)

## 项目概述

本项目是卓越RM机器人的**视觉与导航系统**，集成了人员检测、深度SLAM避障和导航决策功能。基于Berxel P100R 3D相机，使用YOLOv8进行目标检测，结合深度SLAM实现智能导航。

## 主要功能

### 视觉检测
- 🎥 Berxel P100R 3D相机接口
- 🤖 YOLOv8人形检测（**ONNX Runtime加速**）
- 📏 实时深度测量与距离标注
- 🖼️ 深度图平滑与可视化
- ✅ 人形特征验证

### SLAM导航 🆕
- 🗺️ **实时障碍物检测与分割**
- 🎯 **可导航区域分析**
- 🧭 **智能导航决策**
- 📊 **性能监控与统计**
- 🎨 **双窗口实时可视化**

### 性能优化
- 🚀 **AMD 780M 优化（45-80 FPS）**
- ⚡ **SLAM处理（348 FPS）**
- 🔧 **深度图平滑（EMA算法）**

## 文件结构

```
ros2-robt/
├── berxel_camera.py              # Berxel相机Python接口
├── berxel_wrapper.cpp            # Berxel SDK C++包装器
├── person_detect.py              # 人员检测（已优化）
├── person_detect_slam.py         # 🆕 人员检测+SLAM集成
├── depth_slam_obstacle.py        # 🆕 SLAM核心算法
├── test_slam_module.py           # 🆕 SLAM模块测试
├── test_components.py            # 组件测试脚本
├── test_berxel_camera.py         # 相机测试脚本
├── setup.py                      # 编译配置
├── SLAM_INTEGRATION.md           # 🆕 SLAM集成说明
└── SLAM_INTEGRATION_COMPLETE.md  # 🆕 集成完成报告
```

## 依赖项

- Python 3.10+
- OpenCV
- NumPy
- SciPy (SLAM模块)
- **ONNX Runtime**（优化版本）
- Berxel SDK

## 快速开始

### 方式一：运行集成系统（推荐）🆕

```bash
# 1. 激活虚拟环境
source .venv/bin/activate

# 2. 运行人员检测+SLAM导航系统
python person_detect_slam.py

# 控制键:
# q - 退出
# s - 截图
# p - 暂停/继续
# d - 切换SLAM显示
```

### 方式二：运行人员检测（原版）

```bash
# 运行优化版检测程序
python person_detect.py
```

### 方式三：测试SLAM模块

```bash
# 基础测试
python test_slam_module.py --mode basic

# 压力测试
python test_slam_module.py --mode stress
```

## AMD 780M 优化版本（旧版快速开始）

### 1. 安装依赖
```bash
pip install opencv-python numpy onnxruntime
```

### 2. 性能测试（不需要相机）
```bash
python3 test_onnx_performance.py
```

### 3. 完整运行（需要相机）
```bash
python3 person_detect.py
```

**详细说明**：查看 [QUICKSTART_AMD780M.md](QUICKSTART_AMD780M.md)

## 传统安装步骤（PyTorch版本）

1. 安装Python依赖：
```bash
pip install opencv-python numpy ultralytics
```

2. 编译Berxel包装器：
```bash
python3 setup.py build_ext --inplace
```

3. 下载YOLOv8模型：
```bash
# 模型会在首次运行时自动下载
```

## 📊 性能对比（AMD 780M）

| 版本 | 推理引擎 | FPS | 推理时间 | 状态 |
|------|----------|-----|----------|------|
| 原始版 | PyTorch | 13 | ~76ms | 已备份 |
| **优化版** | **ONNX Runtime** | **45-80** | **~12ms** | **✅ 当前** |

**提升倍数**：3.5-6x 🚀

详细报告：[AMD_780M_OPTIMIZATION_REPORT.md](AMD_780M_OPTIMIZATION_REPORT.md)

## 使用方法

### 快速性能测试（推荐）
```bash
./test_amd780m.sh
# 或
python3 test_onnx_performance.py
```

### 运行主程序
```bash
python3 person_detect.py
```

### 快捷键
- `q`: 退出程序

## 配置参数

在 `person_detect.py` 中可以调整以下参数：

### 性能优化参数（AMD 780M）
- `SKIP_FRAMES`: 跳帧数 (默认: 4)
- `YOLO_INPUT_SIZE`: 输入分辨率 (默认: 416)
- `DISPLAY_SCALE`: 显示缩放 (默认: 0.5)
- `DEPTH_PROCESS_INTERVAL`: 深度处理间隔 (默认: 3)

### 检测参数
- `YOLO_CONF_THRESHOLD`: 置信度阈值 (默认: 0.55)
- `MIN_ASPECT_RATIO`: 最小宽高比 (默认: 0.5)
- `MAX_ASPECT_RATIO`: 最大宽高比 (默认: 5.0)

## 技术特点

### 人形验证
- 宽高比检测：1.8 - 3.5
- 面积限制：10000 - 300000 像素
- 置信度阈值：> 0.65
- 边缘检测过滤

### 深度测量
- 实时深度信息显示
- 距离单位：米（m）
- 深度图伪彩色显示

## 开发者

- Zhuo-Skadi

## 许可证

MIT License

## 更新日志

### 2025-10-08
- 初始版本发布
- 实现Berxel相机接口
- 集成YOLOv8检测
- 添加深度图像功能
- 实现人形验证算法
