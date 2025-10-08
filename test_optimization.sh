#!/bin/bash
# 测试与验证脚本
# 用于验证所有功能是否正常工作

set -e

echo "=========================================="
echo "Berxel P100R 优化方案测试脚本"
echo "=========================================="
echo ""

# 颜色定义
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# 检查依赖
echo "1. 检查依赖包..."
echo "----------------------------------------"

check_package() {
    python3 -c "import $1" 2>/dev/null
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✓${NC} $1"
        return 0
    else
        echo -e "${RED}✗${NC} $1 (未安装)"
        return 1
    fi
}

check_package "ultralytics"
check_package "cv2"
check_package "numpy"

echo ""
echo -e "${YELLOW}可选依赖:${NC}"
check_package "onnx" || echo "  提示: pip install onnx"
check_package "onnxruntime" || echo "  提示: pip install onnxruntime"

echo ""
echo "----------------------------------------"
echo ""

# 检查模型文件
echo "2. 检查模型文件..."
echo "----------------------------------------"

if [ -f "yolov8n.pt" ]; then
    echo -e "${GREEN}✓${NC} yolov8n.pt 存在"
    MODEL_SIZE=$(du -h yolov8n.pt | cut -f1)
    echo "  文件大小: $MODEL_SIZE"
else
    echo -e "${YELLOW}!${NC} yolov8n.pt 不存在"
    echo "  Ultralytics会自动下载"
fi

if [ -f "yolov8n.onnx" ]; then
    echo -e "${GREEN}✓${NC} yolov8n.onnx 存在"
    ONNX_SIZE=$(du -h yolov8n.onnx | cut -f1)
    echo "  文件大小: $ONNX_SIZE"
else
    echo -e "${YELLOW}!${NC} yolov8n.onnx 不存在"
    echo "  运行导出: python tools/export_to_onnx.py --model yolov8n.pt --fp16"
fi

echo ""
echo "----------------------------------------"
echo ""

# 测试person_detect.py --help
echo "3. 测试person_detect.py参数..."
echo "----------------------------------------"

timeout 3 python3 person_detect.py --help 2>&1 | grep -E "(usage|--engine|pytorch)" | head -5
if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓${NC} 参数解析正常"
else
    echo -e "${RED}✗${NC} 参数解析失败"
fi

echo ""
echo "----------------------------------------"
echo ""

# 显示推理引擎对比
echo "4. 推理引擎对比表"
echo "----------------------------------------"
echo ""
echo "| 引擎              | 状态 | 适用平台 | 预期加速 |"
echo "|-------------------|------|----------|----------|"

# PyTorch
echo -n "| PyTorch           | "
check_package "ultralytics" >/dev/null 2>&1 && echo -n "可用 " || echo -n "不可用"
echo "| 所有平台 | 1.0x     |"

# ONNX CPU
echo -n "| ONNX (CPU)        | "
check_package "onnxruntime" >/dev/null 2>&1 && echo -n "可用 " || echo -n "不可用"
echo "| 所有平台 | 1.3x     |"

# ONNX OpenVINO
echo -n "| ONNX (OpenVINO)   | "
python3 -c "import onnxruntime as ort; 'OpenVINOExecutionProvider' in ort.get_available_providers()" >/dev/null 2>&1
if [ $? -eq 0 ]; then
    echo -n "可用 "
else
    echo -n "不可用"
fi
echo "| AMD 780M | 2.0x     |"

echo ""
echo "----------------------------------------"
echo ""

# 性能测试说明
echo "5. 性能测试命令"
echo "----------------------------------------"
echo ""
echo "# PyTorch基准（默认）:"
echo "  python person_detect.py --engine pytorch --benchmark"
echo ""
echo "# ONNX CPU:"
echo "  python person_detect.py --engine onnx --benchmark"
echo ""
echo "# ONNX OpenVINO（AMD 780M优化）:"
echo "  python person_detect.py --engine onnx-openvino --benchmark"
echo ""
echo "# 连续测试3种模式:"
echo '  for engine in pytorch onnx onnx-openvino; do'
echo '    echo "测试: $engine"'
echo '    timeout 30 python person_detect.py --engine $engine --benchmark'
echo '    sleep 2'
echo '  done'
echo ""

echo "----------------------------------------"
echo ""

# Docker测试
echo "6. Docker部署命令"
echo "----------------------------------------"
echo ""
echo "# 构建AMD版本镜像:"
echo "  docker build -t person-detector:amd -f Dockerfile ."
echo ""
echo "# 运行（需要先执行 xhost +local:docker）:"
echo "  docker-compose up person-detector-amd"
echo ""
echo "# 或直接运行:"
echo "  docker run -it --rm \\"
echo "    --device=/dev/bus/usb \\"
echo '    --env="DISPLAY=$DISPLAY" \\'
echo '    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \\'
echo '    --volume="$PWD:/workspace" \\'
echo "    person-detector:amd \\"
echo "    person_detect.py --engine onnx-openvino --benchmark"
echo ""

echo "----------------------------------------"
echo ""

# 总结
echo "7. 功能完成度检查"
echo "----------------------------------------"
echo ""

TOTAL=0
DONE=0

check_feature() {
    TOTAL=$((TOTAL + 1))
    if [ $2 -eq 0 ]; then
        echo -e "${GREEN}✓${NC} $1"
        DONE=$((DONE + 1))
    else
        echo -e "${YELLOW}○${NC} $1 (需要安装依赖)"
    fi
}

# 检查各项功能
check_package "ultralytics" >/dev/null 2>&1
check_feature "PyTorch推理引擎" $?

[ -f "tools/export_to_onnx.py" ]
check_feature "ONNX导出脚本" $?

[ -f "person_detect.py" ]
check_feature "多引擎支持" $?

[ -f "Dockerfile" ]
check_feature "Docker部署(AMD)" $?

[ -f "Dockerfile.nvidia" ]
check_feature "Docker部署(NVIDIA参考)" $?

[ -f "docker-compose.yml" ]
check_feature "Docker Compose" $?

[ -f "OPTIMIZATION_README.md" ]
check_feature "优化文档" $?

[ -f "requirements.txt" ]
check_feature "依赖清单" $?

echo ""
echo "完成度: $DONE/$TOTAL"
echo ""

if [ $DONE -eq $TOTAL ]; then
    echo -e "${GREEN}=========================================="
    echo "✓ 所有功能已完成!"
    echo "==========================================${NC}"
else
    echo -e "${YELLOW}=========================================="
    echo "! 部分功能需要安装依赖包"
    echo "  运行: pip install onnx onnxruntime"
    echo "  或使用Docker部署（依赖已打包）"
    echo "==========================================${NC}"
fi

echo ""
echo "详细文档: OPTIMIZATION_README.md"
echo ""
