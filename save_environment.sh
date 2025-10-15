#!/bin/bash
# 环境快照工具 - 保存当前工作环境配置

OUTPUT_DIR="environment_snapshot_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$OUTPUT_DIR"

echo "╔══════════════════════════════════════════════════════════════╗"
echo "║              环境快照工具 - 保存当前配置                       ║"
echo "╚══════════════════════════════════════════════════════════════╝"
echo ""
echo "正在收集环境信息到: $OUTPUT_DIR/"
echo ""

# 1. 系统信息
echo "[1/8] 收集系统信息..."
{
    echo "=== 系统信息 ==="
    uname -a
    echo ""
    cat /etc/os-release
    echo ""
    echo "=== 内核版本 ==="
    uname -r
} > "$OUTPUT_DIR/system_info.txt"

# 2. Python环境
echo "[2/8] 收集Python环境..."
{
    echo "=== Python版本 ==="
    python3 --version
    echo ""
    echo "=== Python路径 ==="
    which python3
    echo ""
    echo "=== 已安装包 ==="
    pip list
} > "$OUTPUT_DIR/python_env.txt"

# 精确版本
pip freeze > "$OUTPUT_DIR/requirements_exact.txt"

# 3. ONNX Runtime配置
echo "[3/8] 收集ONNX Runtime配置..."
{
    echo "=== ONNX Runtime版本 ==="
    python3 -c "import onnxruntime as ort; print(ort.__version__)" 2>&1
    echo ""
    echo "=== 可用Providers ==="
    python3 -c "import onnxruntime as ort; print('\\n'.join(ort.get_available_providers()))" 2>&1
    echo ""
    echo "=== 设备信息 ==="
    python3 -c "import onnxruntime as ort; print(ort.get_device())" 2>&1
} > "$OUTPUT_DIR/onnxruntime_info.txt"

# 4. 硬件信息
echo "[4/8] 收集硬件信息..."
{
    echo "=== CPU信息 ==="
    lscpu | grep -E "Model name|Architecture|CPU\(s\)|Thread|Core"
    echo ""
    echo "=== GPU信息 ==="
    lspci | grep -i "vga\|3d\|display"
    echo ""
    echo "=== 内存信息 ==="
    free -h
} > "$OUTPUT_DIR/hardware_info.txt"

# 5. USB设备
echo "[5/8] 收集USB设备信息..."
{
    echo "=== USB设备列表 ==="
    lsusb
    echo ""
    echo "=== Berxel摄像头 ==="
    lsusb | grep -i "0603:0009"
} > "$OUTPUT_DIR/usb_devices.txt"

# 6. 用户权限
echo "[6/8] 收集用户权限..."
{
    echo "=== 当前用户 ==="
    whoami
    echo ""
    echo "=== 用户组 ==="
    groups
    echo ""
    echo "=== udev规则 ==="
    ls -la /etc/udev/rules.d/ | grep berxel
} > "$OUTPUT_DIR/user_permissions.txt"

# 7. 环境变量
echo "[7/8] 收集环境变量..."
{
    echo "=== LD_LIBRARY_PATH ==="
    echo "$LD_LIBRARY_PATH"
    echo ""
    echo "=== PYTHONPATH ==="
    echo "$PYTHONPATH"
    echo ""
    echo "=== PATH ==="
    echo "$PATH"
} > "$OUTPUT_DIR/environment_vars.txt"

# 8. 运行诊断
echo "[8/8] 运行完整诊断..."
./portable_check.sh > "$OUTPUT_DIR/diagnostic_report.txt" 2>&1 || true

# 创建README
cat > "$OUTPUT_DIR/README.txt" << 'EOF'
# 环境快照说明

这个目录包含了完整的环境配置快照。

## 文件说明

- system_info.txt          : 操作系统和内核信息
- python_env.txt           : Python版本和已安装包
- requirements_exact.txt   : 精确的包版本列表
- onnxruntime_info.txt     : ONNX Runtime配置
- hardware_info.txt        : CPU、GPU、内存信息
- usb_devices.txt          : USB设备列表
- user_permissions.txt     : 用户权限和udev规则
- environment_vars.txt     : 环境变量
- diagnostic_report.txt    : 完整诊断报告

## 如何使用

### 在新机器上恢复环境

1. 安装精确的包版本:
   pip install -r requirements_exact.txt

2. 对比诊断报告:
   ./portable_check.sh > new_diag.txt
   diff diagnostic_report.txt new_diag.txt

3. 根据差异修复问题

### 快速对比

在新环境运行:
  ./portable_check.sh > new_snapshot.txt
  diff diagnostic_report.txt new_snapshot.txt

会显示所有配置差异。

EOF

# 打包
echo ""
echo "正在打包..."
tar -czf "${OUTPUT_DIR}.tar.gz" "$OUTPUT_DIR/"

# 总结
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "✅ 环境快照已创建！"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "📁 快照目录: $OUTPUT_DIR/"
echo "📦 打包文件: ${OUTPUT_DIR}.tar.gz"
echo ""
echo "包含内容:"
ls -lh "$OUTPUT_DIR"
echo ""
echo "打包大小:"
ls -lh "${OUTPUT_DIR}.tar.gz"
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "📝 使用说明："
echo ""
echo "  1. 将 ${OUTPUT_DIR}.tar.gz 复制到新机器"
echo ""
echo "  2. 在新机器上对比:"
echo "     tar -xzf ${OUTPUT_DIR}.tar.gz"
echo "     cd ros2-robt"
echo "     ./portable_check.sh > new_diag.txt"
echo "     diff $OUTPUT_DIR/diagnostic_report.txt new_diag.txt"
echo ""
echo "  3. 根据差异修复问题"
echo ""
