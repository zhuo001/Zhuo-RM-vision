#!/bin/bash
# 安装 ONNX Runtime 加速后端以提升 YOLOv8 推理性能
# 根据硬件选择合适的后端

echo "=========================================="
echo "ONNX Runtime 加速后端安装脚本"
echo "=========================================="
echo ""

# 检测当前硬件
echo "🔍 检测硬件信息..."
CPU_INFO=$(lscpu | grep "Model name" | cut -d: -f2 | xargs)
GPU_INFO=$(lspci | grep -i vga | head -n1)

echo "CPU: $CPU_INFO"
echo "GPU: $GPU_INFO"
echo ""

# 推荐安装策略
if [[ "$CPU_INFO" == *"Intel"* ]]; then
    echo "✅ 检测到 Intel CPU，推荐安装 OpenVINO backend"
    echo ""
    echo "安装命令（选择一个）："
    echo ""
    echo "1. 仅 OpenVINO EP（推荐）："
    echo "   pip install onnxruntime-openvino"
    echo ""
    echo "2. 完整 OpenVINO 工具链（包含模型优化工具）："
    echo "   pip install openvino openvino-dev"
    echo "   # 然后使用 OpenVINOExecutionProvider"
    echo ""
elif [[ "$GPU_INFO" == *"AMD"* ]] || [[ "$CPU_INFO" == *"AMD"* ]]; then
    echo "✅ 检测到 AMD 硬件"
    echo ""
    if [[ "$GPU_INFO" == *"Radeon"* ]]; then
        echo "🎮 集成 GPU (iGPU) 检测，推荐安装 ROCm backend"
        echo ""
        echo "安装命令："
        echo "1. ROCm 运行时（AMD GPU 加速）："
        echo "   # 首先安装 ROCm 驱动（参考 AMD 官方文档）"
        echo "   # https://rocm.docs.amd.com/projects/install-on-linux/en/latest/"
        echo ""
        echo "2. ONNX Runtime ROCm 版本："
        echo "   pip install onnxruntime-rocm"
        echo ""
    fi
    echo "备选方案（CPU 加速，无需 GPU 驱动）："
    echo "   pip install onnxruntime-openvino"
    echo ""
elif [[ "$GPU_INFO" == *"NVIDIA"* ]]; then
    echo "✅ 检测到 NVIDIA GPU"
    echo ""
    echo "安装命令："
    echo "1. CUDA 工具链（需要匹配驱动版本）："
    echo "   # 参考 https://developer.nvidia.com/cuda-downloads"
    echo ""
    echo "2. ONNX Runtime GPU 版本："
    echo "   pip install onnxruntime-gpu"
    echo ""
else
    echo "⚠️ 未能明确识别硬件类型"
    echo ""
    echo "通用加速方案（适用于 Intel/AMD CPU）："
    echo "   pip install onnxruntime-openvino"
    echo ""
fi

echo "=========================================="
echo "当前环境信息"
echo "=========================================="
echo ""
python3 -c "import onnxruntime as ort; print('ONNX Runtime版本:', ort.__version__); print('可用Providers:', ort.get_available_providers())"
echo ""
echo "=========================================="
echo "性能对比（YOLOv8n @ 416x416）"
echo "=========================================="
echo "CPU baseline:    ~11 ms/frame  (~90 FPS 理论, ~45 FPS 实际)"
echo "OpenVINO:        ~6-8 ms/frame (~140 FPS 理论, ~70 FPS 实际)"
echo "ROCm (iGPU):     ~5-7 ms/frame (~160 FPS 理论, ~80 FPS 实际)"
echo "CUDA (dGPU):     ~3-5 ms/frame (~250 FPS 理论, ~120 FPS 实际)"
echo ""
echo "安装加速后端后重新运行 person_detect.py 即可自动启用。"
echo "=========================================="
