# 模型优化与Docker部署指南

本文档说明如何使用ONNX Runtime + OpenVINO优化YOLOv8推理性能，特别适配**AMD 780M核显**。

---

## 📋 目录

1. [快速开始](#快速开始)
2. [推理引擎对比](#推理引擎对比)
3. [AMD 780M优化方案](#amd-780m优化方案推荐)
4. [NVIDIA GPU方案](#nvidia-gpu方案仅供参考)
5. [Docker部署](#docker部署)
6. [性能基准测试](#性能基准测试)
7. [故障排除](#故障排除)

---

## 🚀 快速开始

### 方案选择

| 硬件平台 | 推荐方案 | 预期加速 |
|---------|---------|---------|
| **AMD 780M核显** | ONNXRuntime + OpenVINO | 1.5-2x |
| AMD CPU | ONNXRuntime CPU | 1.2-1.5x |
| NVIDIA GPU | PyTorch (CUDA) | 基准 |
| NVIDIA GPU | TensorRT | 2-4x |

### AMD 780M用户（推荐）

```bash
# 1. 安装依赖
pip install onnx onnxruntime onnxruntime-openvino

# 2. 导出ONNX模型（FP16量化）
python tools/export_to_onnx.py --model yolov8n.pt --fp16

# 3. 使用OpenVINO推理
python person_detect.py --engine onnx-openvino --benchmark
```

---

## 🔍 推理引擎对比

### 1. PyTorch（默认）

**优点:**
- ✅ 零配置，开箱即用
- ✅ 精度最高（FP32）
- ✅ 支持所有硬件

**缺点:**
- ❌ 速度较慢
- ❌ 模型体积大

**使用方法:**
```bash
python person_detect.py --engine pytorch --benchmark
```

### 2. ONNX Runtime (CPU)

**优点:**
- ✅ 跨平台兼容性好
- ✅ 比PyTorch快20-50%
- ✅ 模型体积更小

**缺点:**
- ❌ 首次运行需导出ONNX
- ❌ 不支持动态模型修改

**使用方法:**
```bash
# 导出ONNX
python tools/export_to_onnx.py --model yolov8n.pt --fp16

# 推理
python person_detect.py --engine onnx --benchmark
```

### 3. ONNX Runtime + OpenVINO（AMD 780M推荐）⭐

**优点:**
- ✅ **AMD 780M核显加速**
- ✅ 比PyTorch快50-100%
- ✅ 支持FP16量化
- ✅ CPU/GPU自动选择

**缺点:**
- ❌ 需要额外安装OpenVINO支持

**使用方法:**
```bash
# 安装OpenVINO执行提供器
pip install onnxruntime-openvino

# 导出ONNX（FP16）
python tools/export_to_onnx.py --model yolov8n.pt --fp16

# 使用OpenVINO推理
python person_detect.py --engine onnx-openvino --benchmark
```

### 4. TensorRT（仅NVIDIA GPU）

**优点:**
- ✅ NVIDIA GPU极致优化
- ✅ 2-4倍加速
- ✅ 支持FP16/INT8量化

**缺点:**
- ❌ **仅支持NVIDIA GPU**
- ❌ **AMD 780M无法使用**
- ❌ 编译时间长

---

## 🎯 AMD 780M优化方案（推荐）

### 步骤1: 安装依赖

```bash
# 基础依赖
pip install ultralytics opencv-python numpy

# ONNX支持
pip install onnx onnxruntime

# OpenVINO执行提供器（AMD 780M加速）
pip install onnxruntime-openvino
```

### 步骤2: 导出ONNX模型

```bash
# FP16量化（推荐，模型体积减半，速度更快）
python tools/export_to_onnx.py --model yolov8n.pt --fp16

# FP32（精度最高）
python tools/export_to_onnx.py --model yolov8n.pt

# 查看导出结果
ls -lh yolov8n.onnx
```

**导出参数说明:**
- `--model`: 输入的.pt模型路径
- `--fp16`: 使用FP16量化（推荐）
- `--output`: 自定义输出路径
- `--imgsz`: 输入图像尺寸（默认640）

### 步骤3: 运行推理

```bash
# 使用OpenVINO执行提供器（AMD 780M优化）
python person_detect.py --engine onnx-openvino --benchmark

# 如果OpenVINO不可用，自动回退到CPU
python person_detect.py --engine onnx --benchmark

# 查看帮助
python person_detect.py --help
```

**推理参数说明:**
- `--engine`: 推理引擎（pytorch/onnx/onnx-openvino）
- `--model`: 模型路径（默认yolov8n.pt）
- `--conf`: 置信度阈值（默认0.55）
- `--iou`: NMS IOU阈值（默认0.45）
- `--benchmark`: 显示FPS性能统计

### 步骤4: 性能对比

运行三种模式并记录FPS：

```bash
# PyTorch基准
python person_detect.py --engine pytorch --benchmark

# ONNX CPU
python person_detect.py --engine onnx --benchmark

# ONNX OpenVINO（AMD 780M）
python person_detect.py --engine onnx-openvino --benchmark
```

---

## 🖥️ NVIDIA GPU方案（仅供参考）

> ⚠️ **重要**: AMD 780M用户无法使用此方案，请使用上述OpenVINO方案

### TensorRT优化流程

```bash
# 1. 导出ONNX
python tools/export_to_onnx.py --model yolov8n.pt --fp16 --no-dynamic

# 2. 构建TensorRT引擎（需要NVIDIA GPU）
python tools/onnx_to_tensorrt.py --onnx yolov8n.onnx --fp16

# 3. 使用TensorRT推理（需要实现专用推理脚本）
# 注意: person_detect.py当前不支持TensorRT引擎加载
```

### 环境要求

- NVIDIA GPU（计算能力 >= 6.1）
- CUDA Toolkit >= 11.8
- TensorRT >= 8.6
- nvidia-container-toolkit（Docker）

---

## 🐳 Docker部署

### AMD 780M方案（推荐）

#### 方法1: Docker Compose（最简单）

```bash
# 允许X11显示
xhost +local:docker

# 启动容器
docker-compose up person-detector-amd

# 后台运行
docker-compose up -d person-detector-amd

# 查看日志
docker-compose logs -f person-detector-amd

# 停止
docker-compose down
```

#### 方法2: 直接使用Docker

```bash
# 构建镜像
docker build -t person-detector:amd -f Dockerfile .

# 允许X11显示
xhost +local:docker

# 运行容器
docker run -it --rm \
  --device=/dev/bus/usb \
  --env="DISPLAY=$DISPLAY" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="$PWD:/workspace" \
  person-detector:amd \
  person_detect.py --engine onnx-openvino --benchmark
```

### NVIDIA GPU方案（仅供参考）

```bash
# 构建镜像
docker build -t person-detector:nvidia -f Dockerfile.nvidia .

# 运行容器（需要nvidia-container-toolkit）
docker run -it --rm \
  --gpus all \
  --device=/dev/bus/usb \
  --env="DISPLAY=$DISPLAY" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="$PWD:/workspace" \
  person-detector:nvidia \
  person_detect.py --engine pytorch --benchmark
```

### Docker命令说明

| 参数 | 说明 |
|-----|-----|
| `--device=/dev/bus/usb` | 映射USB设备（Berxel相机） |
| `--env="DISPLAY=$DISPLAY"` | 传递显示环境变量 |
| `--volume="/tmp/.X11-unix:..."` | 映射X11 socket（用于显示窗口） |
| `--volume="$PWD:/workspace"` | 映射项目目录（开发模式） |
| `--gpus all` | 启用GPU（仅NVIDIA） |

---

## 📊 性能基准测试

### 测试环境

- **CPU**: AMD Ryzen 9 7940HS
- **GPU**: AMD Radeon 780M
- **RAM**: 32GB DDR5
- **模型**: YOLOv8n
- **分辨率**: 1920x1080

### 预期性能

| 引擎 | FP32 FPS | FP16 FPS | 加速比 |
|------|----------|----------|--------|
| PyTorch (CPU) | 18-22 | - | 1.0x |
| ONNX (CPU) | 22-28 | 25-32 | 1.3-1.5x |
| ONNX + OpenVINO (780M) | - | 30-40 | 1.7-2.0x |

> 注意: 实际性能取决于场景复杂度、检测框数量等因素

### 运行基准测试

```bash
# 测试脚本（运行60秒）
for engine in pytorch onnx onnx-openvino; do
    echo "测试引擎: $engine"
    timeout 60 python person_detect.py --engine $engine --benchmark 2>&1 | tail -10
    echo "---"
    sleep 2
done
```

---

## 🔧 故障排除

### 问题1: OpenVINO不可用

**现象:**
```
⚠ OpenVINO执行提供器不可用，回退到CPU
提示: 安装OpenVINO支持: pip install onnxruntime-openvino
```

**解决方案:**
```bash
# 安装OpenVINO支持
pip install onnxruntime-openvino

# 或使用官方OpenVINO工具链
# https://docs.openvino.ai/latest/openvino_docs_install_guides_install_dev_tools.html
```

### 问题2: ONNX模型不存在

**现象:**
```
FileNotFoundError: ONNX模型不存在: yolov8n.onnx
请先运行: python tools/export_to_onnx.py --model yolov8n.pt --fp16
```

**解决方案:**
```bash
# 导出ONNX模型
python tools/export_to_onnx.py --model yolov8n.pt --fp16
```

### 问题3: Docker无法访问X11显示

**现象:**
```
Cannot connect to X server
```

**解决方案:**
```bash
# 允许Docker访问X11
xhost +local:docker

# 或使用更安全的方式
xhost +local:`docker inspect --format='{{ .Config.Hostname }}' <container_name>`
```

### 问题4: USB相机无法访问

**现象:**
```
无法初始化Berxel相机
```

**解决方案:**
```bash
# 检查USB设备
lsusb | grep Berxel

# 确保Docker容器有USB访问权限
docker run --device=/dev/bus/usb ...

# 检查udev规则
cat /etc/udev/rules.d/99-berxel.rules
```

### 问题5: NVIDIA GPU不可用（TensorRT）

**现象:**
```
CUDA error: no kernel image is available for execution
```

**解决方案:**
```bash
# 检查NVIDIA驱动
nvidia-smi

# 检查CUDA版本
nvcc --version

# 使用匹配的TensorRT Docker镜像
# 确保TensorRT版本与CUDA版本兼容
```

---

## 📚 参考资源

### AMD 780M优化

- [ONNXRuntime OpenVINO执行提供器文档](https://onnxruntime.ai/docs/execution-providers/OpenVINO-ExecutionProvider.html)
- [OpenVINO工具套件](https://docs.openvino.ai/)
- [AMD GPU推理优化指南](https://www.amd.com/en/technologies/infinity-hub)

### NVIDIA GPU优化（参考）

- [TensorRT开发指南](https://docs.nvidia.com/deeplearning/tensorrt/developer-guide/)
- [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/)
- [YOLOv8 TensorRT部署](https://github.com/ultralytics/ultralytics/blob/main/docs/integrations/tensorrt.md)

### Docker

- [Docker官方文档](https://docs.docker.com/)
- [Docker Compose文档](https://docs.docker.com/compose/)
- [X11 Docker显示转发](https://wiki.ros.org/docker/Tutorials/GUI)

---

## 📝 更新日志

### 2025-01-08
- ✅ 添加ONNXRuntime + OpenVINO支持（AMD 780M优化）
- ✅ 创建ONNX导出工具
- ✅ 添加多推理引擎支持（pytorch/onnx/onnx-openvino）
- ✅ 创建Docker部署方案（AMD/NVIDIA双版本）
- ✅ 添加性能基准测试功能
- ✅ 创建TensorRT转换脚本（NVIDIA参考）

---

## 🤝 贡献

欢迎提交Issue和Pull Request！

特别关注：
- AMD 780M实际性能测试数据
- OpenVINO优化配置
- 其他AMD GPU适配

---

## 📄 许可证

本项目采用MIT许可证。详见LICENSE文件。
