#!/bin/bash
# ROS2 人体检测节点快速启动脚本

# 设置颜色
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}   ROS2 Person Detector Launcher${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# 检查ROS2环境
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${YELLOW}ROS2 环境未加载，正在加载...${NC}"
    source /opt/ros/humble/setup.bash
fi

# 加载工作空间
WORKSPACE_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
echo -e "${GREEN}工作空间: ${WORKSPACE_DIR}${NC}"

if [ -f "${WORKSPACE_DIR}/install/setup.bash" ]; then
    source "${WORKSPACE_DIR}/install/setup.bash"
    echo -e "${GREEN}✓ 工作空间环境已加载${NC}"
else
    echo -e "${YELLOW}警告: install/setup.bash 未找到，请先编译:${NC}"
    echo -e "${YELLOW}  cd ${WORKSPACE_DIR} && colcon build${NC}"
    exit 1
fi

echo ""
echo -e "${BLUE}请选择运行模式:${NC}"
echo -e "  ${GREEN}1${NC} - 基础检测节点（无跟踪）"
echo -e "  ${GREEN}2${NC} - 基础检测节点 + 跟踪"
echo -e "  ${GREEN}3${NC} - 高性能节点（GPU + 多线程 + 跟踪）推荐⭐"
echo -e "  ${GREEN}4${NC} - 测试监听器（查看检测结果）"
echo -e "  ${GREEN}5${NC} - Launch文件启动"
echo ""
read -p "请输入选项 [1-5]: " choice

case $choice in
    1)
        echo -e "${GREEN}启动基础检测节点...${NC}"
        ros2 run person_detector person_detector_node
        ;;
    2)
        echo -e "${GREEN}启动基础检测节点（带跟踪）...${NC}"
        ros2 run person_detector person_detector_node --ros-args -p use_tracking:=true
        ;;
    3)
        echo -e "${GREEN}启动独立高性能检测脚本（非ROS2）...${NC}"
        cd ${WORKSPACE_DIR}/..
        python3 person_detect.py
        ;;
    4)
        echo -e "${GREEN}启动测试监听器...${NC}"
        echo -e "${YELLOW}提示: 请在另一个终端运行检测节点${NC}"
        ros2 run person_detector test_listener
        ;;
    5)
        echo -e "${GREEN}使用Launch文件启动...${NC}"
        ros2 launch person_detector person_detector.launch.py
        ;;
    *)
        echo -e "${YELLOW}无效选项，退出${NC}"
        exit 1
        ;;
esac
