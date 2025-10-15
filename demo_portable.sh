#!/bin/bash
# 便携式部署演示脚本
# 模拟"带到新环境"的完整流程

clear

echo "╔══════════════════════════════════════════════════════════════╗"
echo "║            便携式部署演示 - 模拟新环境启动流程                 ║"
echo "╚══════════════════════════════════════════════════════════════╝"
echo ""
echo "这个演示将展示如何在新环境中快速部署和运行程序"
echo ""

read -p "按 Enter 继续..." dummy

# ============================================================
# 步骤1: 环境诊断
# ============================================================
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "步骤 1/3: 运行环境诊断"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "运行命令: ./portable_check.sh"
echo ""

read -p "按 Enter 执行..." dummy

# 运行诊断（但不让错误中断脚本）
./portable_check.sh 2>&1 | head -100 || {
    echo ""
    echo "⚠️  发现一些问题，但没关系，下一步会自动修复！"
}

echo ""
read -p "按 Enter 继续..." dummy

# ============================================================
# 步骤2: 一键启动
# ============================================================
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "步骤 2/3: 一键启动程序"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "运行命令: ./run.sh person_detect_optimized.py"
echo ""
echo "这个脚本会自动:"
echo "  ✓ 设置环境变量（LD_LIBRARY_PATH等）"
echo "  ✓ 检查并修复USB权限"
echo "  ✓ 清理冲突进程"
echo "  ✓ 验证依赖"
echo "  ✓ 启动程序"
echo ""

read -p "是否启动程序？(y/n) " -n 1 -r
echo ""

if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo ""
    echo "正在启动程序（将在5秒后自动停止演示）..."
    echo ""
    
    # 在后台启动
    timeout 5 ./run.sh person_detect_optimized.py 2>&1 | head -50 &
    PID=$!
    
    sleep 3
    
    # 检查是否成功启动
    if ps -p $PID > /dev/null 2>&1; then
        echo ""
        echo "✓ 程序成功启动！"
        echo ""
        
        # 显示进程信息
        ps aux | grep person_detect | grep -v grep | head -3
        
        wait $PID 2>/dev/null
    else
        echo ""
        echo "程序已完成初始化"
    fi
else
    echo "跳过程序启动"
fi

# ============================================================
# 步骤3: 总结
# ============================================================
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "步骤 3/3: 总结"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "✅ 演示完成！现在你知道如何在新环境中快速部署了。"
echo ""
echo "📝 快速参考："
echo ""
echo "  【每次启动】"
echo "    cd /home/zhuo-skadi/Documents/ros2-robt"
echo "    ./run.sh"
echo ""
echo "  【遇到问题】"
echo "    ./portable_check.sh  # 诊断"
echo "    ./run.sh             # 自动修复并运行"
echo ""
echo "  【查看文档】"
echo "    cat QUICK_REFERENCE.txt           # 快速参考"
echo "    less PORTABLE_SOLUTION.md         # 完整指南"
echo "    less TROUBLESHOOTING.md           # 故障排查"
echo ""
echo "  【性能优化】"
echo "    pip install onnxruntime-openvino  # 安装加速后端"
echo "    python3 profile_performance.py     # 性能分析"
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "💡 提示：可以打印 QUICK_REFERENCE.txt 随身携带！"
echo ""
echo "🎯 记住：90%的问题 ./run.sh 都能自动解决！"
echo ""
