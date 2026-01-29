# Mapless Navigation System - README

## 🎯 项目概述

基于 ROS2 Humble 的无图导航系统，融合多激光雷达和 RGB-D 相机，实现人员跟随功能。

### 传感器配置
- **Unitree L2**: 360° 激光雷达 (50m, 20Hz)
- **Livox Mid-70**: 前向激光雷达 (260m, 10Hz)
- **Berxel P100R**: RGB-D 深度相机 (0.3-5m, 30Hz)

### 核心功能
1. **多传感器点云融合** - PCL 点云拼接、VoxelGrid 降采样
2. **YOLO 目标检测** - ONNX Runtime (ROCm/CUDA/CPU)
3. **ByteTrack 多目标追踪** - 稳定的人员追踪
4. **无图导航** - Nav2 local_costmap 滚动窗口模式
5. **人员跟随** - PID 控制器 + 安全距离维护

## 📦 安装

### 依赖安装

```bash
# ROS2 官方包
sudo apt install ros-humble-nav2-bringup ros-humble-nav2-bt-navigator
sudo apt install ros-humble-pcl-ros ros-humble-pcl-conversions
sudo apt install ros-humble-depth-image-proc ros-humble-cv-bridge
sudo apt install ros-humble-tf2-ros ros-humble-tf2-sensor-msgs
sudo apt install ros-humble-message-filters

# Python 依赖
pip install ultralytics>=8.0.0
pip install onnxruntime>=1.15.0  # 或 onnxruntime-rocm
pip install opencv-python>=4.8.0
pip install numpy scipy
```

### 编译

```bash
cd ~/ros2_ws
colcon build --packages-select mapless_nav
source install/setup.bash
```

## 🚀 启动

### 完整系统启动

```bash
# 启动所有节点 (传感器 + 融合 + 追踪 + 跟随)
ros2 launch mapless_nav mapless_nav.launch.py

# 包含 Nav2 导航栈
ros2 launch mapless_nav mapless_nav.launch.py use_nav2:=true

# 仅可视化
ros2 launch mapless_nav mapless_nav.launch.py enable_following:=false
```

### 单独启动节点

```bash
# 点云融合
ros2 run mapless_nav pointcloud_fusion_node --ros-args --params-file config/fusion_params.yaml

# 深度图转点云
ros2 run mapless_nav depth_to_pointcloud_node

# 目标追踪
ros2 run mapless_nav target_tracker_node.py

# 跟随控制
ros2 run mapless_nav following_controller_node.py
```

## 📋 话题列表

### 输入话题
| 话题 | 类型 | 描述 |
|------|------|------|
| `/unitree_l2/pointcloud` | PointCloud2 | L2 点云 |
| `/livox/lidar` | PointCloud2 | Mid-70 点云 |
| `/berxel/color/image_raw` | Image | RGB 图像 |
| `/berxel/depth/image_raw` | Image | 深度图 |
| `/odom` | Odometry | 里程计 |

### 输出话题
| 话题 | 类型 | 描述 |
|------|------|------|
| `/fused_pointcloud` | PointCloud2 | 融合点云 |
| `/berxel/depth/points` | PointCloud2 | 深度点云 |
| `/target_tracker/tracks` | PoseArray | 所有追踪目标 |
| `/target_tracker/primary_target` | PoseStamped | 主目标位置 |
| `/target_tracker/visualization` | Image | 检测可视化 |
| `/cmd_vel` | Twist | 速度命令 |

## ⚙️ 参数配置

### 点云融合参数 (`config/fusion_params.yaml`)
```yaml
pointcloud_fusion_node:
  voxel_size: 0.05      # 体素大小 (m)
  min_range: 0.3        # 最小距离
  max_range: 15.0       # 最大距离
  enable_ground_removal: false
```

### 追踪参数 (`config/tracker_params.yaml`)
```yaml
target_tracker_node:
  conf_thresh: 0.3      # 检测置信度
  track_thresh: 0.5     # 追踪阈值
  track_buffer: 30      # 丢失缓冲帧数
```

### 跟随参数
```yaml
following_controller_node:
  target_distance: 1.5  # 目标跟随距离 (m)
  min_distance: 0.8     # 最小距离 (停止)
  max_linear_vel: 0.5   # 最大线速度 (m/s)
```

## 🗂️ 文件结构

```
mapless_nav/
├── CMakeLists.txt
├── package.xml
├── setup.py
├── config/
│   ├── nav2_mapless_params.yaml    # Nav2 无图导航配置
│   ├── fusion_params.yaml          # 点云融合配置
│   └── tracker_params.yaml         # 追踪器配置
├── launch/
│   ├── mapless_nav.launch.py       # 主启动文件
│   └── sensors.launch.py           # 传感器启动
├── src/
│   ├── pointcloud_fusion_node.cpp  # C++ 点云融合
│   └── depth_to_pointcloud_node.cpp
├── scripts/
│   ├── target_tracker_node.py      # YOLO + ByteTrack
│   └── following_controller_node.py
├── rviz/
│   └── mapless_nav.rviz
└── mapless_nav/
    └── __init__.py
```

## 📊 性能指标

| 指标 | 目标值 |
|------|--------|
| 检测 FPS | ≥20 |
| 点云融合延迟 | <50ms |
| 追踪准确率 | ≥95% |
| 控制频率 | 20Hz |

## 🔧 故障排除

### 常见问题

1. **ONNX 模型加载失败**
   ```bash
   # 检查模型路径
   ls -la /home/zhuo-skadi/Documents/ros2-robt/yolo12n.onnx
   ```

2. **TF 变换失败**
   ```bash
   # 查看 TF 树
   ros2 run tf2_tools view_frames
   ```

3. **点云不显示**
   ```bash
   # 检查话题
   ros2 topic list | grep pointcloud
   ros2 topic echo /fused_pointcloud --no-arr
   ```

## 📄 许可证

MIT License

## 👥 贡献

欢迎提交 Issue 和 Pull Request!
