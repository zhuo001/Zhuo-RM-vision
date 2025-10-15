#!/bin/bash
# 一键启动脚本 - 自动修复常见问题并启动程序

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "╔══════════════════════════════════════════════════════════════╗"
echo "║              一键启动 - Person Detection                      ║"
echo "║          自动修复常见环境问题并启动程序                        ║"
echo "╚══════════════════════════════════════════════════════════════╝"
echo ""

# ============================================================
# 1. 设置环境变量
# ============================================================
echo "[1/5] 设置环境变量..."

export LD_LIBRARY_PATH="$SCRIPT_DIR/libs:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="$SCRIPT_DIR:${PYTHONPATH:-}"

# 禁用OpenCV的OpenCL（避免某些环境下的冲突）
export OPENCV_OPENCL_RUNTIME=""

# Ensure Qt uses xcb by default to avoid Wayland plugin missing in headless/container envs
export QT_QPA_PLATFORM=${QT_QPA_PLATFORM:-xcb}

echo "  ✓ LD_LIBRARY_PATH: $SCRIPT_DIR/libs"
echo "  ✓ PYTHONPATH: $SCRIPT_DIR"

# ============================================================
# 2. 检查并修复USB权限
# ============================================================
echo ""
echo "[2/5] 检查USB设备权限..."

BERXEL_DEV=$(lsusb | grep "0603:0009" || echo "")
if [ -n "$BERXEL_DEV" ]; then
    echo "  ✓ 检测到Berxel摄像头"
    
    BUS=$(echo "$BERXEL_DEV" | awk '{print $2}')
    DEV=$(echo "$BERXEL_DEV" | awk '{print $4}' | tr -d ':')
    USB_DEV_PATH="/dev/bus/usb/$BUS/$DEV"
    
    if [ ! -r "$USB_DEV_PATH" ] || [ ! -w "$USB_DEV_PATH" ]; then
        echo "  ⚠️  权限不足，尝试修复..."
        sudo chmod 666 "$USB_DEV_PATH" 2>/dev/null && echo "  ✓ 权限已修复" || echo "  ⚠️  需要手动修复权限"
    else
        echo "  ✓ 权限正常"
    fi
else
    echo "  ⚠️  未检测到Berxel摄像头，请检查USB连接"
    read -p "是否继续？(y/n) " -n 1 -r
    echo ""
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# ============================================================
# 3. 清理冲突进程
# ============================================================
echo ""
echo "[3/5] 检查进程冲突..."

CONFLICTING=$(ps aux | grep -E "person_detect|test_camera" | grep python | grep -v grep | grep -v $$ || echo "")
if [ -n "$CONFLICTING" ]; then
    echo "  ⚠️  发现占用摄像头的进程，正在清理..."
    pkill -9 -f "person_detect|test_camera" 2>/dev/null || true
    sleep 1
    echo "  ✓ 已清理"
else
    echo "  ✓ 无冲突进程"
fi

# ============================================================
# 4. 验证依赖
# ============================================================
echo ""
echo "[4/5] 验证Python依赖..."

MISSING_DEPS=()

for pkg in "numpy" "cv2" "onnxruntime"; do
    if ! python3 -c "import $pkg" 2>/dev/null; then
        MISSING_DEPS+=("$pkg")
    fi
done

if [ ${#MISSING_DEPS[@]} -gt 0 ]; then
    echo "  ✗ 缺少依赖: ${MISSING_DEPS[*]}"
    echo "  请运行: pip install -r requirements.txt"
    exit 1
else
    echo "  ✓ 依赖完整"
fi

# ============================================================
# 5. 启动程序
# ============================================================
echo ""
echo "[5/5] 启动程序..."
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

# 检查用户偏好
if [ -f "person_detect_optimized.py" ]; then
    DEFAULT_SCRIPT="person_detect_optimized.py"
else
    DEFAULT_SCRIPT="person_detect.py"
fi

# 允许用户通过参数指定脚本
# 支持 --headless 选项（将其传递给目标脚本）
HEADLESS_FLAG=""
if [ "$1" = "--headless" ]; then
    HEADLESS_FLAG="--headless"
    SCRIPT_TO_RUN="${2:-$DEFAULT_SCRIPT}"
else
    SCRIPT_TO_RUN="${1:-$DEFAULT_SCRIPT}"
fi

if [ ! -f "$SCRIPT_TO_RUN" ]; then
    echo "✗ 错误: $SCRIPT_TO_RUN 不存在"
    exit 1
fi

echo "正在运行: $SCRIPT_TO_RUN $HEADLESS_FLAG"
echo "按 Ctrl+C 或 'q' 退出"
echo ""

# 运行程序
exec python3 "$SCRIPT_TO_RUN" $HEADLESS_FLAG
