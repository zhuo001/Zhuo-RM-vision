# Zhuo-RM

ROS2机器人视觉项目 - 基于Berxel相机和YOLOv8的人形检测系统

## 项目概述

本项目使用Berxel 3D相机和YOLOv8模型实现实时人形检测和深度测量功能。

## 主要功能

- 🎥 Berxel 3D相机接口
- 🤖 YOLOv8人形检测
- 📏 深度信息获取
- 🖼️ 彩色图像和深度图像实时显示
- ✅ 人形验证（宽高比、面积、置信度）

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
- Ultralytics YOLOv8
- Berxel SDK

## 安装步骤

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

## 使用方法

### 测试组件
```bash
python3 test_components.py
```

### 运行主程序
```bash
python3 person_detect.py
```

### 快捷键
- `q`: 退出程序

## 配置参数

在 `person_detect.py` 中可以调整以下参数：

- `model.conf`: 置信度阈值 (默认: 0.45)
- `MIN_ASPECT_RATIO`: 最小宽高比 (默认: 1.8)
- `MAX_ASPECT_RATIO`: 最大宽高比 (默认: 3.5)
- `MIN_AREA`: 最小检测面积 (默认: 10000 像素)
- `MAX_AREA`: 最大检测面积 (默认: 300000 像素)

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
