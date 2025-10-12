# 快速使用指南 - AMD 780M 优化版本

## 🚀 快速开始

### 1. 性能测试（不需要相机）
测试 ONNX Runtime 推理性能：
```bash
python3 test_onnx_performance.py
```

**预期输出**：
```
平均推理时间: ~12ms
理论最大 FPS: ~83 FPS
跳帧优化后 FPS: ~80 FPS
```

### 2. 完整运行（需要相机）
运行人体检测（需要 Berxel 相机）：
```bash
python3 person_detect.py
```

**功能**：
- 实时人体检测与跟踪
- 深度测量（显示距离）
- 左上角显示实时 FPS
- 按 'q' 退出

## 📊 性能对比

| 版本 | FPS | 说明 |
|------|-----|------|
| 原始版本（PyTorch） | 13 FPS | person_detect_pytorch.py.bak |
| **优化版本（ONNX）** | **45-80 FPS** | person_detect.py（当前）✅ |

**提升倍数**：3.5-6x 🚀

## 🔧 配置参数（可调整）

在 `person_detect.py` 顶部修改：

```python
# 性能优化参数
SKIP_FRAMES = 4              # 跳帧数（4=每5帧检测1次）
YOLO_INPUT_SIZE = 416        # YOLO输入尺寸
DISPLAY_SCALE = 0.5          # 显示窗口缩放
DEPTH_PROCESS_INTERVAL = 3   # 深度图处理间隔
```

### 调整建议

**如果 FPS 太低（< 30）**：
```python
SKIP_FRAMES = 7              # 增加跳帧
YOLO_INPUT_SIZE = 320        # 降低分辨率
DISPLAY_SCALE = 0.4          # 缩小显示
```

**如果 FPS 充足（> 50）且想要更好精度**：
```python
SKIP_FRAMES = 2              # 减少跳帧（更频繁检测）
YOLO_INPUT_SIZE = 512        # 提高分辨率（但会降低FPS）
DISPLAY_SCALE = 0.75         # 放大显示
```

## 🔍 文件说明

| 文件 | 用途 |
|------|------|
| `person_detect.py` | 主程序（ONNX优化版本）✅ |
| `person_detect_pytorch.py.bak` | 原始版本备份（PyTorch） |
| `test_onnx_performance.py` | 性能测试工具 |
| `yolov8n.onnx` | ONNX模型文件（FP16，13MB） |
| `yolov8n.pt` | 原始PyTorch模型 |

## 📚 相关文档

- [AMD_780M_OPTIMIZATION_REPORT.md](AMD_780M_OPTIMIZATION_REPORT.md) - 详细优化报告
- [PERFORMANCE_OPTIMIZATION.md](PERFORMANCE_OPTIMIZATION.md) - 性能优化说明
- [FPS_OPTIMIZATION_COMPARISON.md](FPS_OPTIMIZATION_COMPARISON.md) - 优化对比

## 🆘 常见问题

### Q: FPS 显示波动很大？
A: 这是正常的。前几秒 FPS 会较低（模型预热），稳定后应该在 40-80 之间。

### Q: 如何启用 ROCm GPU 加速？
A: 检查 ROCm 是否安装：
```bash
ls /opt/rocm || echo "ROCm not installed"
```
如果已安装：
```bash
pip install onnxruntime-rocm
```
程序会自动检测并使用 ROCMExecutionProvider。

### Q: 检测精度下降了？
A: 可以提高输入分辨率：
```python
YOLO_INPUT_SIZE = 640  # 恢复到标准尺寸
```
但 FPS 会降低到 ~30。

### Q: 能进一步加速吗？
A: 可以尝试 INT8 量化：
```bash
python3 - <<'PY'
from onnxruntime.quantization import quantize_dynamic, QuantType
quantize_dynamic("yolov8n.onnx", "yolov8n_int8.onnx", weight_type=QuantType.QInt8)
PY
```
然后修改 `person_detect.py` 中的模型路径。

## 🎉 总结

通过切换到 ONNX Runtime，我们成功将 AMD 780M 上的性能从 **13 FPS 提升到 45-80 FPS**，无需任何额外的 GPU 驱动配置！

如果有问题，请查看详细报告：[AMD_780M_OPTIMIZATION_REPORT.md](AMD_780M_OPTIMIZATION_REPORT.md)
