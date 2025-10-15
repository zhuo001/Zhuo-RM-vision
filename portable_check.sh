#!/bin/bash
# 便携式环境诊断工具 - 解决"带出去就出问题"

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 问题计数器
ISSUES=0
WARNINGS=0

echo -e "${BLUE}"
echo "╔══════════════════════════════════════════════════════════════╗"
echo "║          便携式环境诊断工具 v1.0                              ║"
echo "║     解决「带出去用就出问题」的环境依赖性问题                    ║"
echo "╚══════════════════════════════════════════════════════════════╝"
echo -e "${NC}"

# ============================================================
# 1. 基础环境检查
# ============================================================
echo -e "\n${BLUE}[1/10] 基础环境检查${NC}"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# 操作系统
OS_INFO=$(cat /etc/os-release | grep PRETTY_NAME | cut -d'"' -f2)
KERNEL=$(uname -r)
echo -e "  操作系统: ${GREEN}$OS_INFO${NC}"
echo -e "  内核版本: $KERNEL"

# Python版本
PYTHON_VER=$(python3 --version 2>&1)
echo -e "  Python: $PYTHON_VER"

# 当前目录
echo -e "  工作目录: $(pwd)"

# ============================================================
# 2. Python依赖检查
# ============================================================
echo -e "\n${BLUE}[2/10] Python依赖检查${NC}"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

REQUIRED_PACKAGES=(
    "numpy"
    "cv2:opencv-python"
    "onnxruntime"
    "ultralytics"
)

for pkg in "${REQUIRED_PACKAGES[@]}"; do
    IFS=':' read -r import_name pip_name <<< "$pkg"
    pip_name=${pip_name:-$import_name}
    
    if python3 -c "import $import_name" 2>/dev/null; then
        version=$(python3 -c "import $import_name; print(getattr($import_name, '__version__', 'unknown'))" 2>/dev/null || echo "unknown")
        echo -e "  ✓ ${GREEN}$import_name${NC} ($version)"
    else
        echo -e "  ✗ ${RED}$import_name 未安装${NC}"
        echo -e "    安装: pip install $pip_name"
        ((ISSUES++))
    fi
done

# 检查ONNX Runtime Providers
echo -e "\n  ONNX Runtime Providers:"
PROVIDERS=$(python3 -c "import onnxruntime as ort; print(','.join(ort.get_available_providers()))" 2>/dev/null || echo "ERROR")
if [ "$PROVIDERS" != "ERROR" ]; then
    IFS=',' read -ra PROVIDER_ARRAY <<< "$PROVIDERS"
    for provider in "${PROVIDER_ARRAY[@]}"; do
        if [[ "$provider" == *"CPU"* ]]; then
            echo -e "    ⚠️  ${YELLOW}$provider${NC} (慢)"
            ((WARNINGS++))
        else
            echo -e "    ✓ ${GREEN}$provider${NC}"
        fi
    done
else
    echo -e "    ✗ ${RED}无法检测${NC}"
    ((ISSUES++))
fi

# ============================================================
# 3. 关键文件检查
# ============================================================
echo -e "\n${BLUE}[3/10] 关键文件检查${NC}"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

REQUIRED_FILES=(
    "yolov8n.onnx:YOLO模型"
    "person_detect.py:主程序"
    "berxel_camera.py:相机模块"
    "berxel_wrapper.cpython-310-x86_64-linux-gnu.so:C++扩展"
    "libs/libBerxelHawk.so:Berxel SDK"
)

for file_info in "${REQUIRED_FILES[@]}"; do
    IFS=':' read -r filepath desc <<< "$file_info"
    
    if [ -f "$filepath" ]; then
        size=$(ls -lh "$filepath" | awk '{print $5}')
        echo -e "  ✓ ${GREEN}$desc${NC} ($filepath, $size)"
    else
        echo -e "  ✗ ${RED}$desc 缺失${NC} ($filepath)"
        ((ISSUES++))
    fi
done

# ============================================================
# 4. USB设备检查
# ============================================================
echo -e "\n${BLUE}[4/10] USB设备检查${NC}"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# 检测Berxel摄像头
BERXEL_DEVICE=$(lsusb | grep "0603:0009" || echo "")
if [ -n "$BERXEL_DEVICE" ]; then
    echo -e "  ✓ ${GREEN}Berxel P100R 已连接${NC}"
    echo "    $BERXEL_DEVICE"
    
    # 提取Bus和Device号
    BUS=$(echo "$BERXEL_DEVICE" | awk '{print $2}')
    DEV=$(echo "$BERXEL_DEVICE" | awk '{print $4}' | tr -d ':')
    USB_DEV_PATH="/dev/bus/usb/$BUS/$DEV"
    
    # 检查设备权限
    if [ -e "$USB_DEV_PATH" ]; then
        PERMS=$(ls -l "$USB_DEV_PATH" | awk '{print $1, $3, $4}')
        echo -e "    设备节点: $USB_DEV_PATH"
        echo -e "    权限: $PERMS"
        
        # 检查是否可读写
        if [ -r "$USB_DEV_PATH" ] && [ -w "$USB_DEV_PATH" ]; then
            echo -e "    ✓ ${GREEN}权限正常${NC}"
        else
            echo -e "    ✗ ${RED}权限不足${NC}"
            echo -e "      修复: sudo chmod 666 $USB_DEV_PATH"
            ((ISSUES++))
        fi
    fi
else
    echo -e "  ✗ ${RED}Berxel P100R 未检测到${NC}"
    echo "    请检查："
    echo "      1. USB线缆是否插好"
    echo "      2. 摄像头是否通电"
    echo "      3. 尝试更换USB端口"
    ((ISSUES++))
fi

# ============================================================
# 5. 用户组权限检查
# ============================================================
echo -e "\n${BLUE}[5/10] 用户组权限检查${NC}"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

CURRENT_USER=$(whoami)
USER_GROUPS=$(groups)

REQUIRED_GROUPS=("video" "berxel")
for group in "${REQUIRED_GROUPS[@]}"; do
    if echo "$USER_GROUPS" | grep -q "\b$group\b"; then
        echo -e "  ✓ 用户在 ${GREEN}$group${NC} 组中"
    else
        echo -e "  ✗ ${YELLOW}用户不在 $group 组中${NC}"
        echo -e "    修复: sudo usermod -aG $group $CURRENT_USER"
        echo -e "    ${RED}修复后需要重新登录！${NC}"
        ((WARNINGS++))
    fi
done

# ============================================================
# 6. udev规则检查
# ============================================================
echo -e "\n${BLUE}[6/10] udev规则检查${NC}"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

UDEV_RULE="/etc/udev/rules.d/berxel-usb.rules"
if [ -f "$UDEV_RULE" ]; then
    echo -e "  ✓ ${GREEN}udev规则已安装${NC} ($UDEV_RULE)"
    
    # 检查是否包含P100R (PID: 0009)
    if grep -q "0009" "$UDEV_RULE"; then
        echo -e "  ✓ ${GREEN}规则包含P100R支持${NC}"
    else
        echo -e "  ✗ ${RED}规则不包含P100R (0009)${NC}"
        ((ISSUES++))
    fi
else
    echo -e "  ✗ ${RED}udev规则未安装${NC}"
    echo -e "    修复: sudo cp berxel-usb.rules /etc/udev/rules.d/"
    echo -e "          sudo udevadm control --reload-rules"
    ((ISSUES++))
fi

# ============================================================
# 7. 动态库检查
# ============================================================
echo -e "\n${BLUE}[7/10] 动态库检查${NC}"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# 检查LD_LIBRARY_PATH
if [ -n "$LD_LIBRARY_PATH" ]; then
    echo -e "  LD_LIBRARY_PATH: $LD_LIBRARY_PATH"
    
    if echo "$LD_LIBRARY_PATH" | grep -q "$(pwd)/libs"; then
        echo -e "  ✓ ${GREEN}libs目录已在LD_LIBRARY_PATH中${NC}"
    else
        echo -e "  ⚠️  ${YELLOW}libs目录不在LD_LIBRARY_PATH中${NC}"
        echo -e "    建议: export LD_LIBRARY_PATH=$(pwd)/libs:\$LD_LIBRARY_PATH"
        ((WARNINGS++))
    fi
else
    echo -e "  ⚠️  ${YELLOW}LD_LIBRARY_PATH 未设置${NC}"
    echo -e "    建议: export LD_LIBRARY_PATH=$(pwd)/libs:\$LD_LIBRARY_PATH"
    ((WARNINGS++))
fi

# 检查关键.so文件
LIBS=(
    "libs/libBerxelHawk.so"
    "libs/libBerxelCommonDriver.so"
    "libs/libBerxelUvcDriver.so"
)

for lib in "${LIBS[@]}"; do
    if [ -f "$lib" ]; then
        # 检查是否可执行
        if ldd "$lib" &>/dev/null; then
            echo -e "  ✓ ${GREEN}$(basename $lib)${NC}"
        else
            echo -e "  ⚠️  ${YELLOW}$(basename $lib) 依赖可能缺失${NC}"
            ((WARNINGS++))
        fi
    else
        echo -e "  ✗ ${RED}$(basename $lib) 缺失${NC}"
        ((ISSUES++))
    fi
done

# ============================================================
# 8. 进程冲突检查
# ============================================================
echo -e "\n${BLUE}[8/10] 进程冲突检查${NC}"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

CONFLICTING_PROCS=$(ps aux | grep -E "person_detect|berxel|test_camera" | grep -v grep | grep -v "$$" || echo "")
if [ -z "$CONFLICTING_PROCS" ]; then
    echo -e "  ✓ ${GREEN}无冲突进程${NC}"
else
    echo -e "  ⚠️  ${YELLOW}发现可能占用摄像头的进程:${NC}"
    echo "$CONFLICTING_PROCS" | while read line; do
        PID=$(echo "$line" | awk '{print $2}')
        CMD=$(echo "$line" | awk '{for(i=11;i<=NF;i++) printf $i" "; print ""}')
        echo -e "    PID $PID: $CMD"
    done
    echo -e "    建议: pkill -9 -f person_detect"
    ((WARNINGS++))
fi

# ============================================================
# 9. 硬件加速检查
# ============================================================
echo -e "\n${BLUE}[9/10] 硬件加速检查${NC}"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# 检测GPU
if lspci | grep -i "vga\|3d\|display" | grep -qi "amd"; then
    GPU_TYPE="AMD"
    echo -e "  GPU: ${GREEN}AMD${NC}"
    
    # 检查ROCm
    if [ -d "/opt/rocm" ]; then
        echo -e "  ✓ ${GREEN}ROCm已安装${NC}"
    else
        echo -e "  ⚠️  ${YELLOW}ROCm未安装，性能受限${NC}"
        echo -e "    安装OpenVINO: pip install onnxruntime-openvino"
        ((WARNINGS++))
    fi
elif lspci | grep -i "vga\|3d\|display" | grep -qi "nvidia"; then
    GPU_TYPE="NVIDIA"
    echo -e "  GPU: ${GREEN}NVIDIA${NC}"
    
    # 检查CUDA
    if command -v nvidia-smi &>/dev/null; then
        echo -e "  ✓ ${GREEN}CUDA驱动已安装${NC}"
    else
        echo -e "  ⚠️  ${YELLOW}CUDA未安装${NC}"
        ((WARNINGS++))
    fi
elif lspci | grep -i "vga\|3d\|display" | grep -qi "intel"; then
    GPU_TYPE="Intel"
    echo -e "  GPU: ${GREEN}Intel${NC}"
    echo -e "  建议安装OpenVINO: pip install onnxruntime-openvino"
else
    echo -e "  GPU: ${YELLOW}未识别${NC}"
fi

# ============================================================
# 10. 网络/外部依赖检查（如果需要）
# ============================================================
echo -e "\n${BLUE}[10/10] 其他检查${NC}"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# 检查磁盘空间
DISK_SPACE=$(df -h . | tail -1 | awk '{print $4}')
echo -e "  可用磁盘空间: $DISK_SPACE"

# 检查内存
MEM_FREE=$(free -h | grep Mem | awk '{print $7}')
echo -e "  可用内存: $MEM_FREE"

# ============================================================
# 总结报告
# ============================================================
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo -e "${BLUE}诊断总结${NC}"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

if [ $ISSUES -eq 0 ] && [ $WARNINGS -eq 0 ]; then
    echo -e "${GREEN}✓ 环境完美！可以正常运行${NC}"
    exit 0
elif [ $ISSUES -eq 0 ]; then
    echo -e "${YELLOW}⚠️  发现 $WARNINGS 个警告（可运行，但性能可能受限）${NC}"
    exit 0
else
    echo -e "${RED}✗ 发现 $ISSUES 个严重问题 + $WARNINGS 个警告${NC}"
    echo ""
    echo "请先修复上述问题，然后重新运行此脚本验证。"
    exit 1
fi
