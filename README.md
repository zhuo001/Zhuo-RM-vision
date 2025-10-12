# Zhuo-RM

ROS2机器人视觉项目 - 基于Berxel相机和YOLOv8的人形检测系统

## 🎉 最新更新（2025-10-11）

**✅ AMD 780M 优化完成！FPS 从 13 提升到 45-80（3.5-6倍提升）**

- 切换到 ONNX Runtime（AMD GPU 兼容）
- 推理速度提升 6.3 倍（76ms → 12ms）
- 详见：[QUICKSTART_AMD780M.md](QUICKSTART_AMD780M.md)

## 项目概述

本项目使用Berxel 3D相机和YOLOv8模型实现实时人形检测和深度测量功能。针对 AMD 780M 集成显卡进行了深度优化。

## 主要功能

- 🎥 Berxel 3D相机接口
- 🤖 YOLOv8人形检测（**ONNX Runtime加速**）
- 📏 深度信息获取
- 🖼️ 彩色图像和深度图像实时显示
- ✅ 人形验证（宽高比、面积、置信度）
- 🚀 **AMD 780M 优化（45-80 FPS）**

## 文件结构

```
ros2-robt/
├── berxel_camera.py       # Berxel相机Python接口
├── berxel_wrapper.cpp     # Berxel SDK C++包装器
├── person_detect.py       # 主检测程序
├── test_components.py     # 组件测试脚本
├── test_berxel_camera.py  # 相机测试脚本
└── setup.py              # 编译配置
```

## 依赖项

- Python 3.10+
- OpenCV
- NumPy
- **ONNX Runtime**（优化版本）
- Berxel SDK

## 快速开始（AMD 780M 优化版本）

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
