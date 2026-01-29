# 🚀 无图导航系统开发计划书

**项目名称**: ROS2 多传感器融合无图导航系统  
**版本**: v1.0  
**日期**: 2026年1月18日  
**负责人**: Zhuo RM Team

---

## 📋 目录

1. [项目概述](#1-项目概述)
2. [系统架构设计](#2-系统架构设计)
3. [硬件配置](#3-硬件配置)
4. [软件技术栈](#4-软件技术栈)
5. [功能模块规划](#5-功能模块规划)
6. [开发阶段与里程碑](#6-开发阶段与里程碑)
7. [技术难点与解决方案](#7-技术难点与解决方案)
8. [风险评估](#8-风险评估)
9. [资源需求](#9-资源需求)
10. [验收标准](#10-验收标准)

---

## 1. 项目概述

### 1.1 项目背景

传统导航系统依赖预建地图，在动态环境、未知场景下灵活性受限。本项目旨在开发一套**无图导航 (Mapless Navigation)** 系统，通过多传感器融合实现实时环境感知与自主导航。

### 1.2 项目目标

构建一套基于 ROS2 的无图导航系统，实现：

- **多激光雷达融合**: Unitree L2 + Livox Mid-70 点云融合
- **深度图转点云**: Berxel P100R RGB-D 深度图转换为点云参与融合
- **实时目标追踪**: YOLO 目标检测 + 追踪算法
- **无图自主导航**: 基于实时感知的路径规划与避障

### 1.3 应用场景

- 室内服务机器人导航
- 动态环境下的人员跟随
- 未知环境探索
- 仓库物流自动化

### 1.4 核心价值

| 特性 | 传统方案 | 本方案 |
|------|----------|--------|
| 地图依赖 | 需预建地图 | 无需地图 |
| 动态适应 | 差 | 强 |
| 部署成本 | 高 | 低 |
| 环境适应性 | 固定环境 | 任意环境 |
| 实时性 | 中等 | 高 |

---

## 2. 系统架构设计

### 2.1 总体架构

```
┌─────────────────────────────────────────────────────────────────────────┐
│                           应用层 (Application)                          │
│  ┌────────────────┐  ┌────────────────┐  ┌────────────────────────────┐│
│  │  人员跟随模式  │  │  自主巡逻模式  │  │     目标追踪模式          ││
│  └────────────────┘  └────────────────┘  └────────────────────────────┘│
└─────────────────────────────────────────────────────────────────────────┘
                                    ↓
┌─────────────────────────────────────────────────────────────────────────┐
│                          决策规划层 (Planning)                          │
│  ┌────────────────┐  ┌────────────────┐  ┌────────────────────────────┐│
│  │ 行为决策器    │  │ 局部路径规划   │  │     全局目标规划          ││
│  │ Behavior Tree │  │ DWA/TEB       │  │     Goal Selector          ││
│  └────────────────┘  └────────────────┘  └────────────────────────────┘│
└─────────────────────────────────────────────────────────────────────────┘
                                    ↓
┌─────────────────────────────────────────────────────────────────────────┐
│                          感知融合层 (Perception)                        │
│  ┌────────────────┐  ┌────────────────┐  ┌────────────────────────────┐│
│  │ 点云融合      │  │ 目标检测识别   │  │     障碍物聚类            ││
│  │ PCL Fusion   │  │ YOLO+Tracking │  │     Euclidean Cluster     ││
│  └────────────────┘  └────────────────┘  └────────────────────────────┘│
│  ┌────────────────┐  ┌────────────────┐  ┌────────────────────────────┐│
│  │ 地面分割      │  │ 可通行区域分析 │  │     动态障碍物追踪        ││
│  │ Ground Seg   │  │ Traversability │  │     Object Tracking       ││
│  └────────────────┘  └────────────────┘  └────────────────────────────┘│
└─────────────────────────────────────────────────────────────────────────┘
                                    ↓
┌─────────────────────────────────────────────────────────────────────────┐
│                          传感器驱动层 (Drivers)                         │
│  ┌────────────────┐  ┌────────────────┐  ┌────────────────────────────┐│
│  │ Unitree L2    │  │ Livox Mid-70  │  │     Berxel P100R          ││
│  │ 360° 2D/3D   │  │ 非重复扫描    │  │     RGB-D → PointCloud   ││
│  │ /unilidar/   │  │ /livox/lidar  │  │     /berxel/depth_cloud   ││
│  └────────────────┘  └────────────────┘  └────────────────────────────┘│
└─────────────────────────────────────────────────────────────────────────┘
                                    ↓
┌─────────────────────────────────────────────────────────────────────────┐
│                          硬件层 (Hardware)                              │
│  ┌────────────────┐  ┌────────────────┐  ┌────────────────────────────┐│
│  │ Unitree L2    │  │ Livox Mid-70  │  │     Berxel P100R RGB-D    ││
│  │ IP: 192.168   │  │ IP: 192.168   │  │     USB 连接              ││
│  │ UDP: 6101     │  │ 以太网        │  │                            ││
│  └────────────────┘  └────────────────┘  └────────────────────────────┘│
└─────────────────────────────────────────────────────────────────────────┘
```

### 2.2 数据流架构

```
┌──────────────────────────────────────────────────────────────────────────┐
│                          实时数据流                                      │
└──────────────────────────────────────────────────────────────────────────┘

  Unitree L2         Mid-70           P100R RGB-D
  (360° 扫描)       (前向扫描)         (深度相机)
       │                │                  │
       ↓                ↓                  ↓
┌────────────┐   ┌────────────┐    ┌────────────────┐
│ /unilidar/ │   │ /livox/    │    │ /berxel/depth  │
│   cloud    │   │   lidar    │    │ + /berxel/rgb  │
└─────┬──────┘   └─────┬──────┘    └───────┬────────┘
      │                │                   │
      │                │           ┌───────┴────────┐
      │                │           │ Depth→PointCloud│
      │                │           │ depth_image_proc│
      │                │           └───────┬────────┘
      │                │                   │
      └────────────────┴───────────────────┘
                       │
               ┌───────▼───────┐
               │  点云融合节点  │
               │ point_cloud_  │
               │   fusion      │
               └───────┬───────┘
                       │
         ┌─────────────┼─────────────┐
         │             │             │
         ↓             ↓             ↓
  ┌────────────┐ ┌────────────┐ ┌────────────┐
  │ 地面分割   │ │ YOLO检测   │ │ 障碍物聚类 │
  └─────┬──────┘ └─────┬──────┘ └─────┬──────┘
        │              │              │
        └──────────────┼──────────────┘
                       │
               ┌───────▼───────┐
               │  感知融合节点  │
               │ perception_   │
               │   fusion      │
               └───────┬───────┘
                       │
               ┌───────▼───────┐
               │  局部代价地图  │
               │ Local Costmap │
               └───────┬───────┘
                       │
               ┌───────▼───────┐
               │  路径规划器    │
               │ DWA/TEB Planner│
               └───────┬───────┘
                       │
               ┌───────▼───────┐
               │   运动控制    │
               │  /cmd_vel     │
               └───────────────┘
```

### 2.3 ROS2 话题设计

```yaml
# 传感器原始数据
/unilidar/cloud:           sensor_msgs/PointCloud2   # Unitree L2
/unilidar/imu:             sensor_msgs/Imu           # L2 IMU
/livox/lidar:              sensor_msgs/PointCloud2   # Mid-70
/berxel/depth:             sensor_msgs/Image         # P100R 深度
/berxel/rgb:               sensor_msgs/Image         # P100R RGB
/berxel/camera_info:       sensor_msgs/CameraInfo    # 相机内参

# 处理后数据
/perception/fused_cloud:   sensor_msgs/PointCloud2   # 融合点云
/perception/ground_cloud:  sensor_msgs/PointCloud2   # 地面点云
/perception/obstacle_cloud:sensor_msgs/PointCloud2   # 障碍物点云
/perception/detections:    vision_msgs/Detection3DArray # 目标检测

# 导航相关
/local_costmap/costmap:    nav_msgs/OccupancyGrid    # 局部代价地图
/local_plan:               nav_msgs/Path             # 局部路径
/cmd_vel:                  geometry_msgs/Twist       # 速度命令

# 目标追踪
/tracking/target:          geometry_msgs/PoseStamped # 追踪目标位置
/tracking/status:          std_msgs/String           # 追踪状态
```

---

## 3. 硬件配置

### 3.1 传感器规格

| 传感器 | 型号 | 关键参数 | 用途 |
|--------|------|----------|------|
| 激光雷达1 | Unitree L2 | 360°, 21600点/秒, 0.05-30m | 全向避障、定位 |
| 激光雷达2 | Livox Mid-70 | 非重复扫描, 100000点/秒, 260m | 前向远距离感知 |
| RGB-D相机 | Berxel P100R | 640×480@30fps, 0.1-5m | 近距离精细感知、目标识别 |

### 3.2 传感器布局

```
                    Mid-70 (前向)
                        │
                       ╱│╲
                      ╱ │ ╲  FOV: 70.4°×77.2°
                     ╱  │  ╲
                    ╱   │   ╲
            ┌──────────────────────┐
            │                      │
   L2 ────→ │       机器人        │ ←──── L2 (360°)
  (左侧)    │       本体          │      (全向)
            │                      │
            │    ┌────────────┐    │
            │    │ P100R RGB-D│    │
            │    │ (前下方)   │    │
            │    └────────────┘    │
            └──────────────────────┘
                    
传感器安装高度:
  - Mid-70: 距地 0.8m, 前倾 10°
  - L2: 距地 0.5m, 水平
  - P100R: 距地 0.4m, 下倾 15°
```

### 3.3 坐标系定义

```
base_link (机器人中心)
    │
    ├── unilidar_link (L2激光雷达)
    │       x: 0.0, y: 0.0, z: 0.5
    │       roll: 0, pitch: 0, yaw: 0
    │
    ├── livox_frame (Mid-70)
    │       x: 0.15, y: 0.0, z: 0.8
    │       roll: 0, pitch: -0.175 (10°下倾), yaw: 0
    │
    └── berxel_link (P100R)
            x: 0.12, y: 0.0, z: 0.4
            roll: 0, pitch: -0.262 (15°下倾), yaw: 0
```

### 3.4 计算平台

| 组件 | 规格 | 用途 |
|------|------|------|
| CPU | AMD Ryzen 7 7840HS | 通用计算、ROS2节点 |
| GPU | AMD 780M | YOLO推理 (ROCm/ONNX) |
| 内存 | 16GB DDR5 | 点云处理缓存 |
| 存储 | 512GB NVMe | 系统、日志 |
| 网络 | 千兆以太网 | 传感器通信 |

---

## 4. 软件技术栈

### 4.1 系统层

```yaml
操作系统: Ubuntu 22.04 LTS
ROS版本: ROS2 Humble Hawksbill
中间件: Fast-DDS
```

### 4.2 核心依赖

```yaml
# 点云处理
PCL: 1.12+                    # 点云库
pcl_ros: ros2 版本            # ROS2 PCL接口
depth_image_proc: ros2        # 深度图转点云

# 导航框架
Nav2: latest                  # ROS2导航栈
nav2_costmap_2d               # 代价地图
nav2_dwb_controller           # DWB局部规划器
nav2_behavior_tree            # 行为树

# 目标检测
ONNX Runtime: 1.16+           # YOLO推理
OpenCV: 4.8+                  # 图像处理
ultralytics: 8.0+             # YOLO模型

# 追踪算法
ByteTrack / DeepSORT          # 多目标追踪

# 传感器驱动
unitree_lidar_ros2            # Unitree L2 驱动
livox_ros_driver2             # Mid-70 驱动
berxel_camera (自研)          # P100R 驱动
```

### 4.3 开发工具

```yaml
构建系统: colcon
代码规范: ament_cpplint, ament_flake8
文档: doxygen, sphinx
测试: pytest, gtest
可视化: RViz2, PlotJuggler
仿真: Gazebo Fortress
```

---

## 5. 功能模块规划

### 5.1 模块清单

```
mapless_nav/                          # 元包
├── mapless_nav_bringup/              # 启动文件
├── mapless_nav_description/          # URDF/坐标系
├── mapless_nav_perception/           # 感知模块
│   ├── pointcloud_fusion/            # 点云融合
│   ├── depth_to_pointcloud/          # 深度转点云
│   ├── ground_segmentation/          # 地面分割
│   ├── obstacle_detection/           # 障碍物检测
│   └── yolo_detection/               # YOLO目标检测
├── mapless_nav_tracking/             # 目标追踪
│   ├── multi_object_tracker/         # 多目标追踪
│   └── person_follower/              # 人员跟随
├── mapless_nav_planning/             # 规划模块
│   ├── local_planner/                # 局部规划
│   └── behavior_tree/                # 行为决策
├── mapless_nav_control/              # 控制模块
└── mapless_nav_interfaces/           # 自定义消息
```

### 5.2 核心模块详解

#### 5.2.1 点云融合模块 (pointcloud_fusion)

**功能**: 融合多源点云为统一坐标系

```cpp
// 伪代码
class PointCloudFusion : public rclcpp::Node {
    // 输入话题
    Subscriber<PointCloud2> sub_l2_;      // Unitree L2
    Subscriber<PointCloud2> sub_mid70_;   // Mid-70
    Subscriber<PointCloud2> sub_p100r_;   // P100R深度转换
    
    // 输出话题
    Publisher<PointCloud2> pub_fused_;
    
    void fuseCallback() {
        // 1. 时间同步 (ApproximateTimeSynchronizer)
        // 2. 坐标转换 (tf2)
        // 3. 体素滤波降采样
        // 4. 点云拼接
        // 5. 发布融合点云
    }
};
```

**参数配置**:
```yaml
pointcloud_fusion:
  ros__parameters:
    # 输入话题
    l2_topic: "/unilidar/cloud"
    mid70_topic: "/livox/lidar"
    p100r_topic: "/berxel/depth_cloud"
    
    # 坐标系
    target_frame: "base_link"
    
    # 滤波参数
    voxel_size: 0.05  # 5cm体素
    
    # 时间同步
    sync_tolerance: 0.1  # 100ms
    
    # 发布频率
    publish_rate: 20.0  # 20Hz
```

#### 5.2.2 深度图转点云模块 (depth_to_pointcloud)

**功能**: 将P100R深度图转换为3D点云

```python
# 使用 depth_image_proc 或自实现
class DepthToPointCloud(Node):
    def __init__(self):
        # 订阅深度图和相机参数
        self.sub_depth = self.create_subscription(
            Image, '/berxel/depth', self.depth_callback, 10)
        self.sub_info = self.create_subscription(
            CameraInfo, '/berxel/camera_info', self.info_callback, 10)
        
        # 发布点云
        self.pub_cloud = self.create_publisher(
            PointCloud2, '/berxel/depth_cloud', 10)
    
    def depth_callback(self, msg):
        # 深度图 → 点云转换
        # 考虑相机内参 fx, fy, cx, cy
        # 过滤无效深度值
        # 发布点云
```

#### 5.2.3 地面分割模块 (ground_segmentation)

**算法选择**: RANSAC / Patchwork++ / Ground Plane Fitting

```cpp
class GroundSegmentation : public rclcpp::Node {
    void cloudCallback(const PointCloud2::SharedPtr msg) {
        // 1. 转换为PCL格式
        // 2. 应用地面分割算法
        //    - RANSAC平面拟合
        //    - 或 Patchwork++ (更鲁棒)
        // 3. 分离地面点和非地面点
        // 4. 发布分割结果
    }
};
```

#### 5.2.4 YOLO目标检测模块 (yolo_detection)

**功能**: 实时检测人员、车辆等目标

```python
class YoloDetector(Node):
    def __init__(self):
        # 加载ONNX模型
        self.session = ort.InferenceSession(
            'yolov8n.onnx', 
            providers=['ROCMExecutionProvider', 'CPUExecutionProvider']
        )
        
        # 订阅RGB图像
        self.sub_rgb = self.create_subscription(
            Image, '/berxel/rgb', self.rgb_callback, 10)
        
        # 发布检测结果
        self.pub_detections = self.create_publisher(
            Detection2DArray, '/perception/detections_2d', 10)
    
    def rgb_callback(self, msg):
        # 1. 图像预处理 (resize, normalize)
        # 2. YOLO推理
        # 3. 后处理 (NMS)
        # 4. 发布检测框
```

#### 5.2.5 多目标追踪模块 (multi_object_tracker)

**算法**: ByteTrack / DeepSORT

```python
class MultiObjectTracker(Node):
    def __init__(self):
        self.tracker = ByteTracker()  # 或 DeepSORT
        
        # 订阅2D检测和深度信息
        self.sub_det = message_filters.Subscriber(
            self, Detection2DArray, '/perception/detections_2d')
        self.sub_depth = message_filters.Subscriber(
            self, Image, '/berxel/depth')
        
        # 时间同步
        self.ts = ApproximateTimeSynchronizer(
            [self.sub_det, self.sub_depth], 10, 0.1)
        self.ts.registerCallback(self.sync_callback)
        
        # 发布3D追踪结果
        self.pub_tracks = self.create_publisher(
            Detection3DArray, '/tracking/tracks', 10)
    
    def sync_callback(self, det_msg, depth_msg):
        # 1. 2D检测 + 深度 → 3D位置
        # 2. ByteTrack关联与追踪
        # 3. 发布追踪结果
```

#### 5.2.6 无图局部规划器 (local_planner)

**方案**: 基于 Nav2 DWB/TEB + 自定义代价层

```yaml
# nav2_params.yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      # 无地图模式: 仅使用局部代价地图
      
local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 10.0
      publish_frequency: 5.0
      global_frame: odom
      robot_base_frame: base_link
      rolling_window: true
      width: 5.0
      height: 5.0
      resolution: 0.05
      
      plugins: ["obstacle_layer", "inflation_layer"]
      
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        observation_sources: fused_cloud
        fused_cloud:
          topic: /perception/obstacle_cloud
          data_type: PointCloud2
          marking: true
          clearing: true
          max_obstacle_height: 2.0
          min_obstacle_height: 0.1
```

#### 5.2.7 人员跟随模块 (person_follower)

**功能**: 识别并跟随特定人员

```python
class PersonFollower(Node):
    def __init__(self):
        self.target_id = None  # 追踪目标ID
        self.follow_distance = 1.5  # 跟随距离(米)
        
        # 订阅追踪结果
        self.sub_tracks = self.create_subscription(
            Detection3DArray, '/tracking/tracks', self.tracks_callback, 10)
        
        # 发布目标位置 (供导航使用)
        self.pub_goal = self.create_publisher(
            PoseStamped, '/tracking/target', 10)
        
        # 发布速度命令 (简单跟随模式)
        self.pub_cmd = self.create_publisher(
            Twist, '/cmd_vel', 10)
    
    def tracks_callback(self, msg):
        # 1. 查找目标ID的追踪结果
        # 2. 计算目标相对位置
        # 3. 生成跟随目标点 (保持距离)
        # 4. 发布目标或直接控制
```

---

## 6. 开发阶段与里程碑

### 6.1 开发阶段

```
┌─────────────────────────────────────────────────────────────────────────┐
│  阶段1: 基础设施 (第1-2周)                                              │
│  ├─ 搭建ROS2工作空间                                                    │
│  ├─ 配置传感器驱动                                                      │
│  ├─ 创建URDF/TF树                                                       │
│  └─ 验证各传感器数据                                                    │
├─────────────────────────────────────────────────────────────────────────┤
│  阶段2: 点云处理 (第3-4周)                                              │
│  ├─ 实现深度图转点云                                                    │
│  ├─ 实现多源点云融合                                                    │
│  ├─ 实现地面分割                                                        │
│  └─ 障碍物聚类与提取                                                    │
├─────────────────────────────────────────────────────────────────────────┤
│  阶段3: 目标感知 (第5-6周)                                              │
│  ├─ YOLO目标检测集成                                                    │
│  ├─ 2D到3D投影                                                          │
│  ├─ 多目标追踪实现                                                      │
│  └─ 目标状态估计                                                        │
├─────────────────────────────────────────────────────────────────────────┤
│  阶段4: 导航规划 (第7-8周)                                              │
│  ├─ 局部代价地图配置                                                    │
│  ├─ DWB/TEB规划器调优                                                   │
│  ├─ 行为树设计                                                          │
│  └─ 人员跟随逻辑                                                        │
├─────────────────────────────────────────────────────────────────────────┤
│  阶段5: 集成测试 (第9-10周)                                             │
│  ├─ 系统集成                                                            │
│  ├─ 性能优化                                                            │
│  ├─ 场景测试                                                            │
│  └─ Bug修复                                                             │
├─────────────────────────────────────────────────────────────────────────┤
│  阶段6: 文档与交付 (第11-12周)                                          │
│  ├─ 用户文档                                                            │
│  ├─ API文档                                                             │
│  ├─ 部署指南                                                            │
│  └─ 演示与培训                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

### 6.2 里程碑定义

| 里程碑 | 时间 | 交付物 | 验收标准 |
|--------|------|--------|----------|
| M1 | 第2周末 | 传感器驱动就绪 | 所有传感器数据正常发布 |
| M2 | 第4周末 | 点云融合完成 | 融合点云质量达标，FPS≥10 |
| M3 | 第6周末 | 目标追踪就绪 | 人员追踪准确率≥90% |
| M4 | 第8周末 | 导航功能完成 | 无碰撞自主导航 |
| M5 | 第10周末 | 系统集成完成 | 通过集成测试 |
| M6 | 第12周末 | 项目交付 | 文档完善，演示成功 |

### 6.3 甘特图

```
任务                     W1   W2   W3   W4   W5   W6   W7   W8   W9   W10  W11  W12
─────────────────────────────────────────────────────────────────────────────────────
基础设施搭建            ████ ████
传感器驱动配置          ████ ████
URDF/TF配置                  ████
深度图转点云                      ████
点云融合                          ████ ████
地面分割                               ████
障碍物检测                              ████ ████
YOLO集成                                     ████
多目标追踪                                   ████ ████
局部代价地图                                           ████
路径规划器                                             ████ ████
行为决策                                                    ████
人员跟随                                                    ████
系统集成                                                          ████ ████
性能优化                                                               ████
测试验证                                                                    ████
文档编写                                                                    ████ ████
─────────────────────────────────────────────────────────────────────────────────────
里程碑          M1              M2              M3              M4         M5    M6
```

---

## 7. 技术难点与解决方案

### 7.1 多传感器时间同步

**难点**: 不同传感器时钟不同步，数据时间戳不一致

**解决方案**:
1. 使用 `message_filters::ApproximateTimeSynchronizer`
2. 设置合理的同步容差 (50-100ms)
3. 使用系统时间统一时间戳
4. 必要时实现自定义同步器

### 7.2 多传感器空间标定

**难点**: 不同传感器坐标系需要精确标定

**解决方案**:
1. 使用标定板进行外参标定
2. 工具: `lidar_camera_calibration` / `kalibr`
3. 验证: 点云与图像对齐检查
4. 在URDF中精确定义TF关系

### 7.3 点云融合质量

**难点**: 融合后可能出现重影、漂移

**解决方案**:
1. ICP点云配准
2. 体素滤波去重
3. 统计滤波去噪
4. 动态调整融合权重

### 7.4 实时性保证

**难点**: 多模块并行，计算资源竞争

**解决方案**:
1. 合理分配CPU核心 (isolcpus)
2. 使用ROS2组件化架构
3. 降低非关键模块频率
4. GPU加速YOLO推理
5. 点云降采样

### 7.5 动态障碍物处理

**难点**: 移动障碍物难以准确预测

**解决方案**:
1. 多目标追踪 (ByteTrack)
2. 速度估计与预测
3. 动态代价层
4. 反应式避障

### 7.6 无图定位

**难点**: 无地图情况下的定位

**解决方案**:
1. 使用里程计 (wheel + IMU)
2. 视觉里程计 (VO)
3. 激光里程计 (LO)
4. 融合定位 (EKF)

---

## 8. 风险评估

### 8.1 风险矩阵

| 风险 | 可能性 | 影响 | 等级 | 缓解措施 |
|------|--------|------|------|----------|
| 传感器驱动不稳定 | 中 | 高 | 🔴 | 使用稳定版本，充分测试 |
| 时间同步问题 | 高 | 中 | 🔴 | 预留调试时间，备选方案 |
| 计算资源不足 | 中 | 高 | 🔴 | 算法优化，降级策略 |
| 标定精度不足 | 中 | 中 | 🟡 | 多次标定验证 |
| 动态环境干扰 | 高 | 中 | 🟡 | 鲁棒算法，参数调优 |
| 进度延误 | 中 | 中 | 🟡 | 预留缓冲时间 |

### 8.2 应急预案

```
风险触发 → 评估影响 → 启动应急预案 → 调整计划
                              │
                    ┌─────────┼─────────┐
                    ↓         ↓         ↓
              降级方案    备选技术   增加资源
```

---

## 9. 资源需求

### 9.1 硬件资源

| 设备 | 数量 | 用途 | 状态 |
|------|------|------|------|
| Unitree L2 | 1 | 360°扫描 | ✅ 已有 |
| Livox Mid-70 | 1 | 前向感知 | ✅ 已有 |
| Berxel P100R | 1 | RGB-D | ✅ 已有 |
| 计算平台 (AMD 780M) | 1 | 运算 | ✅ 已有 |
| 机器人底盘 | 1 | 移动平台 | ⚠️ 待确认 |
| 测试场地 | 1 | 功能验证 | ⚠️ 待确认 |

### 9.2 软件资源

| 软件 | 版本 | 授权 | 状态 |
|------|------|------|------|
| Ubuntu | 22.04 | 免费 | ✅ |
| ROS2 Humble | 最新 | Apache 2.0 | ✅ |
| Nav2 | 最新 | Apache 2.0 | ✅ |
| PCL | 1.12+ | BSD | ✅ |
| ONNX Runtime | 1.16+ | MIT | ✅ |
| YOLOv8 模型 | n/s | AGPL | ✅ |

### 9.3 人力资源

| 角色 | 人数 | 职责 |
|------|------|------|
| 项目负责人 | 1 | 总体协调、架构设计 |
| 感知开发 | 1 | 点云处理、目标检测 |
| 导航开发 | 1 | 规划、控制 |
| 测试工程师 | 1 | 测试、验证 |

---

## 10. 验收标准

### 10.1 功能验收

| 功能 | 验收标准 | 测试方法 |
|------|----------|----------|
| 传感器数据采集 | 所有传感器正常发布 | 话题检查 |
| 点云融合 | 融合点云无明显错位 | 可视化检查 |
| 地面分割 | 分割准确率>95% | 人工标注对比 |
| 目标检测 | mAP>70%, FPS>15 | 标准测试集 |
| 目标追踪 | MOTA>80%, 稳定追踪 | 多场景测试 |
| 避障导航 | 无碰撞通过障碍区 | 实际场景测试 |
| 人员跟随 | 稳定跟随5分钟 | 实际场景测试 |

### 10.2 性能验收

| 指标 | 目标值 | 测试方法 |
|------|--------|----------|
| 系统延迟 | <200ms | 端到端测量 |
| 点云融合FPS | ≥10 | ROS2工具 |
| 目标检测FPS | ≥15 | 计时统计 |
| CPU占用 | <80% | htop监控 |
| 内存占用 | <8GB | htop监控 |
| 导航成功率 | >95% | 多次测试统计 |

### 10.3 文档验收

| 文档 | 内容要求 |
|------|----------|
| 用户手册 | 安装、配置、使用说明 |
| API文档 | 所有公开接口说明 |
| 部署指南 | 环境搭建、依赖安装 |
| 架构文档 | 系统设计、模块说明 |
| 测试报告 | 测试用例、结果、问题 |

---

## 📎 附录

### A. 参考项目

待搜索的GitHub项目类型:
1. ROS2 无图导航 (mapless navigation)
2. 多激光雷达融合 (multi-lidar fusion)
3. 深度图转点云 (depth to pointcloud)
4. YOLO ROS2集成 (yolo ros2)
5. 人员跟随 (person following)
6. ByteTrack/DeepSORT ROS2

### B. 技术文档链接

- [Nav2 官方文档](https://navigation.ros.org/)
- [PCL ROS2](https://github.com/ros-perception/perception_pcl)
- [Unitree L2 SDK](https://github.com/unitreerobotics/unilidar_sdk)
- [Livox ROS2 Driver](https://github.com/Livox-SDK/livox_ros_driver2)

### C. 联系方式

- 项目仓库: zhuo001/Zhuo-RM-Main
- 当前分支: fix/depth-smoothing

---

**文档版本**: 1.0  
**创建日期**: 2026-01-18  
**最后更新**: 2026-01-18  
**状态**: 📋 计划中

---

> 💡 **下一步**: 搜索GitHub相关开源项目，选择最佳技术方案，开始开发！
