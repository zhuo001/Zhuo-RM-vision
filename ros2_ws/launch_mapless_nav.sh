#!/bin/bash
# 无图导航系统启动脚本
# Mapless Navigation System Launcher

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 路径
ROS2_WS=~/Documents/ros2-robt/ros2_ws
SDK_LIB=$HOME/.local/lib

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}  无图导航系统 - Mapless Navigation${NC}"
echo -e "${BLUE}========================================${NC}"

# 设置环境
echo -e "${YELLOW}[1/4] 设置环境变量...${NC}"
source /opt/ros/humble/setup.bash
source $ROS2_WS/install/setup.bash
export LD_LIBRARY_PATH=$SDK_LIB:$LD_LIBRARY_PATH

# 检查包
echo -e "${YELLOW}[2/4] 检查 ROS2 包...${NC}"
REQUIRED_PKGS=("mapless_nav" "berxel_camera_ros2" "livox_ros_driver2")
for pkg in "${REQUIRED_PKGS[@]}"; do
    if ros2 pkg list | grep -q "^$pkg$"; then
        echo -e "  ${GREEN}✓${NC} $pkg"
    else
        echo -e "  ${RED}✗${NC} $pkg (未找到)"
    fi
done

# 显示可用的启动选项
echo -e "${YELLOW}[3/4] 可用启动模式:${NC}"
echo -e "  ${GREEN}1${NC}. 完整系统 (mapless_nav + 所有传感器)"
echo -e "  ${GREEN}2${NC}. 仅 mapless_nav (点云融合 + 目标追踪)"
echo -e "  ${GREEN}3${NC}. 仅 Berxel 相机"
echo -e "  ${GREEN}4${NC}. 仅 Livox (需要连接雷达)"
echo ""

# 读取用户选择
read -p "请选择启动模式 [1-4]: " choice

echo -e "${YELLOW}[4/4] 启动系统...${NC}"

case $choice in
    1)
        echo -e "${GREEN}启动完整系统...${NC}"
        # TODO: 创建组合 launch 文件
        ros2 launch mapless_nav mapless_nav.launch.py enable_visualization:=false
        ;;
    2)
        echo -e "${GREEN}启动 mapless_nav...${NC}"
        ros2 launch mapless_nav mapless_nav.launch.py enable_visualization:=false
        ;;
    3)
        echo -e "${GREEN}启动 Berxel 相机...${NC}"
        ros2 launch berxel_camera_ros2 berxel_camera.launch.py
        ;;
    4)
        echo -e "${GREEN}启动 Livox 雷达...${NC}"
        ros2 launch livox_ros_driver2 rviz_MID360_launch.py
        ;;
    *)
        echo -e "${RED}无效选择${NC}"
        exit 1
        ;;
esac
