# Zhuo-RM

ROS2机器人视觉与导航项目 - 基于Berxel相机、YOLOv12检测与深度SLAM导航

## 🎉 最新更新

### 2025-12-15: 模型升级至 YOLOv12 🚀
**✅ 检测模型全面升级！**
- ✅ 升级至 YOLOv12 Nano 模型 (`yolo12n.onnx`)
- ✅ 保持 ONNX Runtime 加速支持
- ✅ 更高的检测精度与速度平衡

### 2025-12-15: Unitree L2 LiDAR 集成完成 🎯
**✅ 3D LiDAR 点云可视化系统上线！**
- ✅ 集成 Unitree L2 LiDAR（ROS 2 Humble）
- ✅ 实时点云鸟瞰图显示（±20m 范围）
- ✅ 三窗口可视化（检测 | 深度/SLAM | LiDAR点云）
- ✅ 高度着色与机器人位置指示
- ✅ ROS 2 话题订阅 `/unilidar/cloud`
- ✅ 自动参数优化（work_mode=1）

### 2025-10-17: SLAM导航集成完成 ✨
**✅ 深度SLAM障碍物检测与导航决策系统上线！**
- ✅ 实时障碍物检测（348 FPS处理速度）
- ✅ 可导航区域分析
- ✅ 智能导航决策（forward/left/right/stop）
- ✅ 双窗口可视化界面
- ✅ ROS2集成准备就绪
- 详见：[SLAM_INTEGRATION.md](SLAM_INTEGRATION.md)

### 2025-10-15: 深度图闪烁修复 🔧
- ✅ EMA时间平滑算法
- ✅ 深度可视化持久化
- ✅ 检测深度采样优化

### 2025-10-11: AMD 780M 性能优化 🚀
- ✅ FPS 从 13 提升到 45-80（3.5-6倍提升）
- ✅ ONNX Runtime GPU 加速
- 详见：[QUICKSTART_AMD780M.md](QUICKSTART_AMD780M.md)

## 项目概述

本项目是卓越RM机器人的**视觉与导航系统**，集成了人员检测、深度SLAM避障和导航决策功能。基于Berxel P100R 3D相机，使用YOLOv12进行目标检测，结合深度SLAM实现智能导航。

## 主要功能

### 视觉检测
- 🎥 Berxel P100R 3D相机接口
- 🤖 YOLOv12人形检测（**ONNX Runtime加速**）
- 📏 实时深度测量与距离标注
- 🖼️ 深度图平滑与可视化
- ✅ 人形特征验证

### SLAM导航 🆕
- 🗺️ **实时障碍物检测与分割**
- 🎯 **可导航区域分析**
- 🧭 **智能导航决策**
- 📊 **性能监控与统计**
- 🎨 **三窗口实时可视化**（检测+深度+LiDAR）

### LiDAR集成 🌟
- 📡 **Unitree L2 LiDAR 支持**
- 🗺️ **实时点云鸟瞰图（400x400）**
- 🎨 **高度着色显示**（地面/障碍物/高物体）
- 🤖 **机器人位置与朝向指示**
- 🔧 **ROS 2 话题订阅**（`/unilidar/cloud`）

### 性能优化
- 🚀 **AMD 780M 优化（45-80 FPS）**
- ⚡ **SLAM处理（348 FPS）**
- 🔧 **深度图平滑（EMA算法）**

## 文件结构

```
ros2-robt/
├── berxel_camera.py              # Berxel相机Python接口
├── berxel_wrapper.cpp            # Berxel SDK C++包装器
├── person_detect.py              # 人员检测（已优化）
├── person_detect_slam.py         # � 人员检测+SLAM+LiDAR集成
├── depth_slam_obstacle.py        # 🆕 SLAM核心算法
├── test_slam_module.py           # 🆕 SLAM模块测试
├── debug_pointcloud.py           # 🌟 点云调试工具
├── test_udp.py                   # 🌟 LiDAR UDP测试
├── test_lidar_params.py          # 🌟 LiDAR参数扫描
├── test_components.py            # 组件测试脚本
├── test_berxel_camera.py         # 相机测试脚本
├── setup.py                      # 编译配置
├── ros2_ws/                      # 🌟 ROS 2 工作空间
│   ├── launch_sensors.py         # 传感器启动文件
│   └── run_sensors.sh            # 快捷启动脚本
├── SLAM_INTEGRATION.md           # 🆕 SLAM集成说明
└── SLAM_INTEGRATION_COMPLETE.md  # 🆕 集成完成报告
```

## 依赖项

- Python 3.10+
- OpenCV
- NumPy
- SciPy (SLAM模块)
- **ONNX Runtime**（优化版本）
- Berxel SDK
- **ROS 2 Humble**（LiDAR集成）
- **rclpy**（ROS 2 Python客户端库）

## 快速开始

### 方式一：运行完整集成系统（推荐）🌟

**需要 Berxel 相机 + Unitree L2 LiDAR**

```bash
# 1. 启动 LiDAR（终端1）
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 run unitree_lidar_ros2 unitree_lidar_ros2_node \
  --ros-args \
  -p initialize_type:=2 \
  -p work_mode:=1 \
  -p lidar_ip:="192.168.1.1" \
  -p local_ip:="192.168.1.2" \
  -p lidar_port:=6101 \
  -p local_port:=6201

# 2. 运行集成系统（终端2）
source .venv/bin/activate
python person_detect_slam.py

# 控制键:
# q - 退出
# s - 截图
# p - 暂停/继续
# d - 切换SLAM显示
```

**显示窗口**：
- 左侧：YOLOv12 人员检测 + RGB
- 中间：深度图 + SLAM 导航可视化
- 右侧：LiDAR 点云鸟瞰图（±20m）

### 方式二：运行人员检测（原版）

```bash
# 运行优化版检测程序
python person_detect.py
```

### 方式三：测试SLAM模块

```bash
# 基础测试
python test_slam_module.py --mode basic

# 压力测试
python test_slam_module.py --mode stress
```

## AMD 780M 优化版本（旧版快速开始）

### 1. 安装依赖
```bash
pip install opencv-python numpy onnxruntime
```

### 2. 性能测试（不需要相机）
```bash
python3 test_onnx_performance.py
```

### 3. 完整运行（需要相机）
```bash
python3 person_detect.py
```

**详细说明**：查看 [QUICKSTART_AMD780M.md](QUICKSTART_AMD780M.md)

## 传统安装步骤（PyTorch版本）

1. 安装Python依赖：
```bash
pip install opencv-python numpy ultralytics
```

2. 编译Berxel包装器：
```bash
python3 setup.py build_ext --inplace
```

3. 下载YOLOv8模型：
```bash
# 模型会在首次运行时自动下载
```

## 📊 性能对比（AMD 780M）

| 版本 | 推理引擎 | FPS | 推理时间 | 状态 |
|------|----------|-----|----------|------|
| 原始版 | PyTorch | 13 | ~76ms | 已备份 |
| **优化版** | **ONNX Runtime** | **45-80** | **~12ms** | **✅ 当前** |

**提升倍数**：3.5-6x 🚀

详细报告：[AMD_780M_OPTIMIZATION_REPORT.md](AMD_780M_OPTIMIZATION_REPORT.md)

## 使用方法

### 快速性能测试（推荐）
```bash
./test_amd780m.sh
# 或
python3 test_onnx_performance.py
```

### 运行主程序
```bash
python3 person_detect.py
```

### 快捷键
- `q`: 退出程序

## 配置参数

在 `person_detect_slam.py` 中可以调整以下参数：

### 性能优化参数（AMD 780M）
- `SKIP_FRAMES`: 跳帧数 (默认: 1)
- `YOLO_INPUT_SIZE`: 输入分辨率 (默认: 416)
- `DISPLAY_SCALE`: 显示缩放 (默认: 0.5)
- `DEPTH_PROCESS_INTERVAL`: 深度处理间隔 (默认: 3)

### 检测参数
- `YOLO_CONF_THRESHOLD`: 置信度阈值 (默认: 0.40)
- `YOLO_IOU_THRESHOLD`: NMS IoU阈值 (默认: 0.45)
- `MIN_ASPECT_RATIO`: 最小宽高比 (默认: 0.3)
- `MAX_ASPECT_RATIO`: 最大宽高比 (默认: 8.0)

### LiDAR 配置参数 🌟
在 `person_detect_slam.py` 的 `pointcloud2_to_birdview()` 函数中：
- `width`, `height`: 鸟瞰图尺寸 (默认: 400x400)
- `resolution`: 空间分辨率 (默认: 0.02 = 2cm/像素)
- **有效范围**: ±20m x ±20m（由 resolution × width 决定）
- **高度阈值**: 地面 -0.2m, 高物体 0.5m

### Unitree L2 LiDAR 运行参数
- `initialize_type`: 2（标准初始化）
- `work_mode`: **1**（关键！默认0会导致立即退出）
- `lidar_ip`: 192.168.1.1（LiDAR固定IP）
- `local_ip`: 192.168.1.2（本机需配置为此IP）
- `lidar_port`: 6101（LiDAR数据端口）
- `local_port`: 6201（本机接收端口）

## 技术特点

### 人形验证
- 宽高比检测：1.8 - 3.5
- 面积限制：10000 - 300000 像素
- 置信度阈值：> 0.65
- 边缘检测过滤

### 深度测量
- 实时深度信息显示
- 距离单位：米（m）
- 深度图伪彩色显示

### LiDAR 点云处理 🌟
- **分辨率**: 0.02m/像素（400x400鸟瞰图）
- **覆盖范围**: ±20m x ±20m
- **高度着色**:
  - 🔴 红色: z < -0.2m（坑洼/低于地面）
  - 🟢 绿色: -0.2m ≤ z ≤ 0.5m（正常地面/小障碍物）
  - 🟡 黄色: z > 0.5m（高障碍物/墙壁）
- **坐标系**: 机器人位于图像下方中心，前方向上
- **性能**: ~30 FPS 点云可视化

## 开发者

- Zhuo-Skadi

## 许可证

MIT License

## 故障排查 🔧

### LiDAR 相关问题

**问题1：LiDAR 节点持续重启/立即退出**
```bash
# 解决方案：确保 work_mode=1
ros2 run unitree_lidar_ros2 unitree_lidar_ros2_node \
  --ros-args -p work_mode:=1
```

**问题2：点云窗口为黑色/显示 "No LiDAR Data"**
```bash
# 1. 检查 LiDAR 节点是否运行
ps aux | grep unitree_lidar_ros2_node

# 2. 检查话题是否发布
ros2 topic list | grep unilidar
ros2 topic hz /unilidar/cloud

# 3. 检查网络连接
ping 192.168.1.1
python3 test_udp.py  # 监听UDP数据包

# 4. 查看 person_detect_slam.py 调试输出
# 应该看到 "LiDAR Debug: Total X points, Valid Y points"
```

**问题3：本机 IP 配置**
```bash
# LiDAR 要求本机 IP 为 192.168.1.2
sudo ip addr add 192.168.1.2/24 dev <你的网口名>
# 例如: sudo ip addr add 192.168.1.2/24 dev eth0
```

### 调试工具

- `test_udp.py`: 测试 LiDAR UDP 数据接收
- `debug_pointcloud.py`: 独立点云话题监听（需要 LiDAR 不被占用）
- `test_lidar_params.py`: 自动扫描 LiDAR 参数组合

## 更新日志

### 2025-12-15
- 🌟 集成 Unitree L2 LiDAR 点云可视化
- 🌟 三窗口实时显示（检测+深度+点云）
- 🌟 实现鸟瞰图转换与高度着色
- 🔧 修复 LiDAR 节点重启问题（work_mode=1）
- 🔧 优化点云解析，支持可变 point_step
- 📝 添加 ROS 2 启动脚本和调试工具

### 2025-10-17
- 🆕 SLAM导航集成完成
- 🆕 实时障碍物检测与导航决策

### 2025-10-15
- 🔧 深度图闪烁修复（EMA算法）
- 🔧 深度可视化持久化

### 2025-10-11
- 🚀 AMD 780M 性能优化（45-80 FPS）
- ⚡ ONNX Runtime GPU 加速

### 2025-10-08
- 初始版本发布
- 实现Berxel相机接口
- 集成YOLOv8检测
- 添加深度图像功能
- 实现人形验证算法
