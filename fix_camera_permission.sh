#!/bin/bash
# Berxel P100R 摄像头权限修复脚本

echo "🔧 Berxel P100R 摄像头权限修复工具"
echo "=========================================="
echo ""

# 检测摄像头
echo "📷 检测Berxel摄像头..."
BERXEL_DEV=$(lsusb | grep -i "0603:0009")

if [ -z "$BERXEL_DEV" ]; then
    echo "❌ 未检测到Berxel P100R摄像头 (VID:0603 PID:0009)"
    echo "   请检查USB连接"
    exit 1
fi

echo "✓ 检测到摄像头: $BERXEL_DEV"
echo ""

# 检查当前用户组
echo "👤 检查用户组..."
CURRENT_USER=$(whoami)
USER_GROUPS=$(groups)

if echo "$USER_GROUPS" | grep -q "berxel"; then
    echo "✓ 用户已在 berxel 组中"
else
    echo "⚠️  用户不在 berxel 组中"
fi

if echo "$USER_GROUPS" | grep -q "video"; then
    echo "✓ 用户已在 video 组中"
else
    echo "⚠️  用户不在 video 组中"
fi

echo ""

# 检查udev规则
echo "📋 检查udev规则..."
if [ -f "/etc/udev/rules.d/berxel-usb.rules" ]; then
    echo "✓ udev规则文件存在"
    
    if grep -q "0009" /etc/udev/rules.d/berxel-usb.rules; then
        echo "✓ 规则包含P100R (PID: 0009)"
    else
        echo "❌ 规则不包含P100R产品ID"
    fi
else
    echo "❌ udev规则文件不存在"
fi

echo ""
echo "=========================================="
echo "🛠️  修复方案"
echo "=========================================="
echo ""

# 方案1: 重新插拔USB
echo "方案 1: 重新插拔USB设备"
echo "  1. 拔出Berxel摄像头USB线"
echo "  2. 等待3秒"
echo "  3. 重新插入USB线"
echo "  4. 等待设备初始化（约5秒）"
echo ""

# 方案2: 重新加载udev规则
echo "方案 2: 重新加载udev规则并触发"
echo "  运行以下命令:"
echo "  $ sudo udevadm control --reload-rules"
echo "  $ sudo udevadm trigger"
echo "  $ sudo chmod 666 /dev/bus/usb/001/005  # 临时方案"
echo ""

# 方案3: 直接修改设备权限（临时）
BUS=$(echo "$BERXEL_DEV" | awk '{print $2}')
DEV=$(echo "$BERXEL_DEV" | awk '{print $4}' | tr -d ':')

echo "方案 3: 直接修改设备权限（临时，重启失效）"
echo "  $ sudo chmod 666 /dev/bus/usb/$BUS/$DEV"
echo ""

# 方案4: 添加用户到组（需要重新登录）
echo "方案 4: 确保用户在正确的组（需重新登录）"
echo "  $ sudo usermod -aG berxel $CURRENT_USER"
echo "  $ sudo usermod -aG video $CURRENT_USER"
echo "  然后注销并重新登录"
echo ""

# 方案5: 重启libusb服务
echo "方案 5: 重置USB子系统"
echo "  $ sudo rmmod uvcvideo"
echo "  $ sudo modprobe uvcvideo"
echo ""

echo "=========================================="
echo "🚀 快速修复（推荐）"
echo "=========================================="
echo ""

read -p "是否执行快速修复？(y/n) " -n 1 -r
echo ""

if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo "执行修复..."
    echo ""
    
    # 1. 重新加载udev规则
    echo "[1/4] 重新加载udev规则..."
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    echo "✓ 完成"
    
    # 2. 临时修改设备权限
    echo "[2/4] 修改设备权限..."
    USB_DEV="/dev/bus/usb/$BUS/$DEV"
    if [ -e "$USB_DEV" ]; then
        sudo chmod 666 "$USB_DEV"
        echo "✓ 完成: $USB_DEV"
    else
        echo "⚠️  设备节点不存在: $USB_DEV"
    fi
    
    # 3. 添加用户到组
    echo "[3/4] 添加用户到berxel和video组..."
    sudo usermod -aG berxel "$CURRENT_USER"
    sudo usermod -aG video "$CURRENT_USER"
    echo "✓ 完成"
    
    # 4. 检查libusb权限
    echo "[4/4] 设置libusb权限..."
    if [ -f "/usr/lib/x86_64-linux-gnu/libusb-1.0.so" ]; then
        sudo chmod 755 /usr/lib/x86_64-linux-gnu/libusb-1.0.so*
        echo "✓ 完成"
    fi
    
    echo ""
    echo "=========================================="
    echo "✅ 修复完成！"
    echo "=========================================="
    echo ""
    echo "下一步："
    echo "1. 重新插拔USB摄像头"
    echo "2. 运行测试: python3 person_detect.py"
    echo "3. 如果还不行，请注销并重新登录（使组权限生效）"
    echo ""
    
    # 显示当前设备权限
    echo "当前设备权限:"
    ls -l "$USB_DEV" 2>/dev/null || echo "  设备节点不存在"
    echo ""
    
else
    echo "跳过自动修复"
fi

echo ""
echo "📝 如果问题仍然存在，请尝试:"
echo "   1. 重启计算机"
echo "   2. 更换USB端口"
echo "   3. 检查USB线缆是否损坏"
echo "   4. 查看dmesg日志: dmesg | tail -50"
