# ONNX Runtime 加速后端优化指南

## 概述
本项目已升级为智能推理后端选择，可自动检测并使用以下加速器（按优先级）：
1. **OpenVINO** - Intel CPU/iGPU 优化（推荐 Intel 平台）
2. **ROCm** - AMD GPU 加速（适用于 AMD Radeon）
3. **CUDA** - NVIDIA GPU 加速
4. **CPU** - 基线实现（所有平台）

**当前状态**：使用 CPU baseline（~11 ms/frame，实际 ~45 FPS）

---

## 快速安装（根据硬件选择）

### Intel 平台（推荐）
```bash
# 方案1：仅安装 OpenVINO EP（轻量级，推荐）
pip install onnxruntime-openvino

# 方案2：完整 OpenVINO 工具链（含模型优化工具）
pip install openvino openvino-dev
```

**预期性能提升**：6-8 ms/frame（~70 FPS 实际，提升 55%）

---

### AMD 平台（当前系统）

#### 选项A：OpenVINO CPU 加速（最简单）
```bash
pip install onnxruntime-openvino
```
- ✅ 无需 GPU 驱动
- ✅ 一键安装
- ⚡ 性能提升 ~50%（70 FPS）

#### 选项B：ROCm GPU 加速（最佳性能）
```bash
# 1. 安装 ROCm 驱动（参考 AMD 官方文档）
# https://rocm.docs.amd.com/projects/install-on-linux/en/latest/
# Ubuntu 22.04 示例：
wget https://repo.radeon.com/amdgpu-install/latest/ubuntu/jammy/amdgpu-install_6.3.60300-1_all.deb
sudo apt install ./amdgpu-install_6.3.60300-1_all.deb
sudo amdgpu-install --usecase=rocm

# 2. 安装 ONNX Runtime ROCm 版本
pip install onnxruntime-rocm
```
- ⚡ 性能提升 ~80%（80+ FPS）
- ⚠️ 需要安装 ROCm 驱动（复杂度较高）

---

### NVIDIA 平台
```bash
# 1. 安装 CUDA 工具链（需匹配驱动版本）
# https://developer.nvidia.com/cuda-downloads

# 2. 安装 ONNX Runtime GPU 版本
pip install onnxruntime-gpu
```

**预期性能提升**：3-5 ms/frame（~120 FPS 实际，提升 167%）

---

## 验证安装

安装后运行以下命令确认后端可用：

```bash
python3 -c "import onnxruntime as ort; print('Available:', ort.get_available_providers())"
```

**预期输出示例**：
- CPU: `['AzureExecutionProvider', 'CPUExecutionProvider']`
- OpenVINO: `['OpenVINOExecutionProvider', 'CPUExecutionProvider']`
- ROCm: `['ROCMExecutionProvider', 'CPUExecutionProvider']`
- CUDA: `['CUDAExecutionProvider', 'CPUExecutionProvider']`

---

## 性能对比表

| 后端 | 推理时间 | 理论FPS | 实际FPS | 相对提升 | 安装难度 |
|------|----------|---------|---------|----------|----------|
| CPU | 11 ms | 90 | 45 | baseline | ✅ 默认 |
| OpenVINO | 6-8 ms | 140 | 70 | +55% | ✅ 简单 |
| ROCm | 5-7 ms | 160 | 80 | +78% | ⚠️ 中等 |
| CUDA | 3-5 ms | 250 | 120 | +167% | ⚠️ 中等 |

*实际 FPS 包含预处理、后处理、显示等开销*

---

## 自动检测逻辑

`person_detect.py` 启动时会自动按优先级选择可用后端：

```
检测流程：
1. ✓ 检查 OpenVINO → 使用（Intel 优化）
2. ✓ 检查 ROCm → 使用（AMD GPU）
3. ✓ 检查 CUDA → 使用（NVIDIA GPU）
4. ✓ 回退到 CPU（兜底方案）
```

**无需修改代码**，安装加速包后自动启用。

---

## 故障排查

### OpenVINO 未检测到
```bash
# 确认安装
pip list | grep onnx

# 重新安装
pip uninstall onnxruntime onnxruntime-openvino
pip install onnxruntime-openvino
```

### ROCm Provider 不可用
```bash
# 检查 ROCm 安装
rocm-smi

# 检查环境变量
echo $ROCM_PATH

# 验证设备
/opt/rocm/bin/rocminfo
```

### 性能未提升
- 确认 `person_detect.py` 启动日志显示正确后端
- 运行 `test_onnx_performance.py` 验证纯推理速度
- 检查系统负载（CPU/GPU 占用率）

---

## 推荐配置（当前系统）

**硬件**：AMD Ryzen + Radeon iGPU

**推荐方案**：
1. **短期**（5分钟安装）：
   ```bash
   pip install onnxruntime-openvino
   ```
   - 性能提升：~50%（70 FPS）
   - 无需 GPU 驱动

2. **长期**（最佳性能）：
   - 安装 ROCm 驱动
   - 使用 `onnxruntime-rocm`
   - 性能提升：~80%（80+ FPS）

---

## 参考资源

- [ONNX Runtime Execution Providers](https://onnxruntime.ai/docs/execution-providers/)
- [OpenVINO 安装指南](https://docs.openvino.ai/latest/openvino_docs_install_guides_overview.html)
- [ROCm 安装指南](https://rocm.docs.amd.com/projects/install-on-linux/en/latest/)
- [AMD 780M 优化报告](./AMD_780M_OPTIMIZATION_REPORT.md)

---

## 使用示例

安装加速后端后，直接运行：

```bash
python3 person_detect.py
```

启动时会显示：
```
🔍 Detecting available execution providers...
Available providers: ['OpenVINOExecutionProvider', 'CPUExecutionProvider']
✅ Using OpenVINOExecutionProvider (Intel CPU/iGPU acceleration)
   ➜ OpenVINO provides optimized inference for Intel hardware

📦 Loading model: yolov8n.onnx
✓ Model loaded successfully
  Input: images [1, 3, 416, 416]
  Output: ['output0']
  Active backend: OpenVINO
  Session providers: ['OpenVINOExecutionProvider', 'CPUExecutionProvider']
```

观察 FPS 提升即可验证加速效果！
