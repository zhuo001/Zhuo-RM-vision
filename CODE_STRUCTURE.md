# 代码整理与功能说明文档

**项目名称**: 机器人感知与导航集成系统  
**更新时间**: 2026年1月18日  
**状态**: 准备整合版本

---

## 1. 项目总体架构

```
┌─────────────────────────────────────────────────────────┐
│          实时感知与导航决策系统（集成版本）              │
└─────────────────────────────────────────────────────────┘
         ↓         ↓          ↓          ↓
    ┌────────────────────────────────────────┐
    │         传感器驱动层                   │
    │  ├─ Berxel相机(RGB-D)                  │
    │  ├─ Mid-70激光雷达(ROS2)               │
    │  └─ 内置IMU/里程计                     │
    └────────────────────────────────────────┘
         ↓         ↓          ↓
    ┌────────────────────────────────────────┐
    │         算法处理层                     │
    │  ├─ YOLO目标检测(人体/物体)            │
    │  ├─ 深度SLAM障碍物分析                 │
    │  └─ 点云可视化处理                     │
    └────────────────────────────────────────┘
         ↓         ↓          ↓
    ┌────────────────────────────────────────┐
    │         决策与可视化层                 │
    │  ├─ 融合决策输出                       │
    │  ├─ 统一界面显示                       │
    │  └─ 导航指引生成                       │
    └────────────────────────────────────────┘
```

---

## 2. 核心模块详解

### 2.1 传感器驱动模块

#### 2.1.1 Berxel相机驱动 (Python)
**文件**: `berxel_camera.py`  
**功能**: RGB-D深度相机的初始化与数据获取  
**主要类/函数**:
- `BerxelCamera` - 相机管理类
  - `initialize()` - 初始化相机
  - `get_frame()` - 获取RGB彩色帧（返回BGR格式）
  - `get_depth()` - 获取深度图（单位：毫米）
  - `release()` - 释放资源

**依赖**: `berxel_wrapper` (C++ 扩展模块)

---

#### 2.1.2 Berxel包装层 (C++)
**文件**: `berxel_wrapper.cpp`  
**功能**: 连接Python与Berxel SDK的C++桥接层  
**编译**:
```bash
python setup.py build_ext --inplace
```

**相关文件**:
- `setup.py` - 编译配置
- `Include/` - Berxel SDK头文件
  - `BerxelHawkDevice.h` - 设备管理
  - `BerxelHawkFrame.h` - 帧数据结构
  - `BerxelHawkContext.h` - 上下文管理
  - `BerxelHawkDefines.h` - 常量定义

---

#### 2.1.3 激光雷达驱动测试 (Python)
**文件**: `test_lidar_params.py`  
**功能**: Mid-70激光雷达参数配置与验证  
**主要功能**:
- UDP数据包检测
- 参数化ROS2节点启动
- 不同工作模式测试
- 网络连接验证

**注意**: 该脚本用于调试，生产环境应使用ROS2启动文件

---

### 2.2 目标检测模块

#### 2.2.1 基础YOLO检测 (Python)
**文件**: `person_detect.py`  
**功能**: 基于ONNX Runtime的实时人体检测  
**核心参数**:
- 模型: `yolo12n.onnx` (轻量级模型)
- 输入分辨率: 416×416 (为AMD 780M优化)
- 置信度阈值: 0.40 (平衡检出率与误报)
- NMS阈值: 0.45
- 跳帧: 1 (每帧检测，最流畅)

**执行后端**:
```python
# 自动选择优先级:
1. ROCMExecutionProvider (AMD GPU)
2. CUDAExecutionProvider (NVIDIA GPU)
3. CPUExecutionProvider (CPU)
```

**主要处理流程**:
1. 获取Berxel相机RGB帧
2. YOLO推理 (ONNX Runtime)
3. 后处理（NMS去重）
4. 目标坐标与置信度输出
5. 实时显示与记录

---

#### 2.2.2 传统方法检测 (Python)
**文件**: `berxel_person_detect.py`  
**功能**: 基于Ultralytics YOLOv8的目标检测  
**特点**:
- 使用原生PyTorch模型 (`yolov8n.pt`)
- 仅检测人体 (`classes=[0]`)
- 获取中心点深度信息
- 计算目标实际尺寸

**性能**: 略低于ONNX版本，但无需模型转换

---

#### 2.2.3 ONNX模型工具 (Python)
**文件**: `tools/export_to_onnx.py`  
**功能**: PyTorch模型转ONNX格式  
**用途**: 优化推理性能，支持跨平台部署

**相关工具**:
- `tools/onnx_to_tensorrt.py` - ONNX转TensorRT
- `tools/benchmark_engines.py` - 性能基准测试
- `tools/benchmark_openvino.py` - OpenVINO基准测试

---

### 2.3 SLAM与障碍物检测模块

#### 2.3.1 深度SLAM障碍物检测 (Python)
**文件**: `depth_slam_obstacle.py`  
**类**: `DepthSLAMObstacleDetector`  

**核心参数**:
- 近距离阈值: 0.5m (视为障碍物)
- 远距离阈值: 5.0m (忽略)
- 最小障碍物高度: 0.1m
- 栅格分辨率: 0.05m
- 最小可导航区域: 1000像素

**主要方法**:
```python
# 处理深度帧
obstacle_mask, info = detector.process_depth_frame(depth_meters, color_frame)

# 返回信息包含:
{
    'suggested_direction': 'forward'|'left'|'right'|'blocked',
    'obstacle_count': int,
    'navigable_zones': [zone1, zone2, ...],
    'min_depth': float,
    'processing_time': float,
    'confidence': float
}
```

**功能划分**:
1. **深度预处理**: 高斯滤波、无效值处理
2. **障碍物检测**: 阈值分割、形态学处理
3. **可导航区域分析**: 连通域标记、质心计算
4. **决策生成**: 最优方向选择

**可视化**:
- 红色: 障碍物区域
- 绿色: 可导航区域
- 灰色: 未知区域

---

#### 2.3.2 集成人体检测+SLAM (Python)
**文件**: `person_detect_slam.py` (主要整合文件)  
**功能**: 结合YOLO人体检测与SLAM障碍物分析  

**工作流程**:
```
Berxel相机(RGB+Depth)
         ↓
    ┌────────────┐
    │ RGB帧      │──→ YOLO推理 → 人体检测结果
    │ 深度图     │──→ SLAM处理 → 障碍物分析
    └────────────┘
         ↓
    融合决策层 (融合两个结果)
         ↓
    ┌────────────────────────────────┐
    │ 输出: 目标位置 + 导航方向        │
    │ 可视化: 统一显示界面            │
    └────────────────────────────────┘
```

**参数配置** (针对AMD 780M优化):
```python
SKIP_FRAMES = 1           # 每帧处理（无跳帧）
YOLO_INPUT_SIZE = 416     # 降低分辨率加速
DISPLAY_SCALE = 0.5       # 50%显示缩放
TARGET_FPS = 30           # 目标帧率
DEPTH_PROCESS_INTERVAL = 3 # SLAM每3帧处理1次
```

**时间序列平滑**:
- YOLO结果缓存与趋势分析
- 深度EMA平滑 (α=0.25)
- 可视化帧缓存 (5帧)

---

### 2.4 点云可视化模块

#### 2.4.1 Mid-70激光雷达可视化 (Python)
**文件**: `mid70_vis.py`  
**功能**: ROS2点云话题实时可视化  

**工作原理**:
1. 创建ROS2节点 `mid70_visualizer`
2. 订阅话题 `/livox/lidar` (可配置)
3. 接收PointCloud2消息
4. 实时转换为Open3D格式
5. 弹窗显示3D点云

**可配置参数**:
```python
topic_name = '/livox/lidar'  # 可修改为其他话题
window_size = (1280, 720)    # 窗口大小
```

**依赖**:
- `rclpy` (ROS2 Python客户端)
- `sensor_msgs_py` (消息转换)
- `open3d` (3D可视化)

**使用方法**:
```bash
# 确保激光雷达驱动已运行
source /opt/ros/humble/setup.bash
python3 mid70_vis.py
```

---

#### 2.4.2 点云调试工具 (Python)
**文件**: `debug_pointcloud.py`  
**功能**: PointCloud2消息内容诊断  

**输出信息**:
- 消息头信息 (frame_id, 时间戳)
- 点云尺寸 (height, width)
- 字段定义 (offset, datatype)
- 首个点坐标
- 数据统计

**用途**: 验证点云话题格式是否正确

---

### 2.5 测试与验证模块

#### 2.5.1 SLAM模块测试 (Python)
**文件**: `test_slam_module.py`  
**功能**:
- 模拟深度图生成
- 障碍物检测验证
- 性能基准测试 (100帧连续处理)
- 统计信息输出

**运行**:
```bash
python3 test_slam_module.py
```

---

#### 2.5.2 相机测试 (Python)
**文件**: `test_berxel_camera.py`  
**功能**: Berxel相机驱动验证  
**检查项**:
- 相机初始化成功
- RGB帧获取正常
- 深度图数据有效
- 帧率是否达标

---

#### 2.5.3 YOLO模型测试 (Python)
**文件**: `test_yolo.py`  
**功能**: 验证YOLOv8模型可用性  

---

#### 2.5.4 其他测试脚本
- `test_camera.py` - 通用相机测试
- `test_depth.py` - 深度图处理测试
- `test_udp.py` - UDP通信测试
- `test_onnx_performance.py` - ONNX推理基准测试

---

### 2.6 ROS2工作区

#### 2.6.1 启动脚本
**文件**: `ros2_ws/launch_sensors.py`  
**功能**: ROS2 Python启动脚本  
**启动的节点**:
- Berxel RGB-D驱动
- Mid-70激光雷达驱动
- 点云订阅节点

**使用**:
```bash
cd ros2_ws
colcon build
source install/setup.bash
ros2 launch launch_sensors.py
```

---

## 3. 依赖关系图

```
person_detect_slam.py (整合核心)
    ├─ berxel_camera.py
    │   └─ berxel_wrapper (C++)
    │       └─ Berxel SDK libs
    ├─ depth_slam_obstacle.py
    │   └─ OpenCV, SciPy, NumPy
    ├─ ONNX Runtime
    │   ├─ ROCm (AMD)
    │   ├─ CUDA (NVIDIA, 不推荐)
    │   └─ CPU
    └─ ROS2 (可选)
        ├─ rclpy
        ├─ sensor_msgs
        └─ cv_bridge

mid70_vis.py
    ├─ ROS2
    │   └─ sensor_msgs_py
    ├─ Open3D
    └─ NumPy

test_*.py
    ├─ 对应模块
    ├─ OpenCV
    └─ NumPy

Tools (工具集)
    ├─ ultralytics (YOLO导出)
    ├─ ONNX
    └─ TensorRT (可选)
```

---

## 4. 性能优化指标 (AMD 780M)

| 指标 | 当前值 | 目标值 | 备注 |
|------|--------|--------|------|
| YOLO推理 | ~30ms | <33ms | 416×416 ONNX |
| SLAM处理 | ~20ms | <30ms | 每3帧1次 |
| 点云可视化 | ~15ms | <30ms | Open3D渲染 |
| 总体FPS | 25-28 | 30+ | 整合版本 |
| GPU利用 | 60-70% | 70-80% | ROCm优化空间 |

---

## 5. 整合版本规划

### 5.1 主程序架构 (`unified_system.py` - 待创建)

```python
class UnifiedRobotSystem:
    """统一机器人感知系统"""
    
    def __init__(self):
        # 初始化所有模块
        self.camera = BerxelCamera()           # RGB-D
        self.yolo_detector = YOLODetector()    # 人体检测
        self.slam = DepthSLAMObstacleDetector() # 障碍物
        self.visualizer = UnifiedVisualizer()   # 显示界面
        self.decision_maker = DecisionMaker()   # 决策层
        
    def run(self):
        """主循环"""
        while True:
            # 1. 获取传感器数据
            rgb, depth = self.camera.get_data()
            
            # 2. 并行处理
            person_detections = self.yolo_detector.detect(rgb)
            obstacle_info = self.slam.process(depth, rgb)
            
            # 3. 融合决策
            nav_cmd = self.decision_maker.fuse(person_detections, obstacle_info)
            
            # 4. 显示与输出
            self.visualizer.render(rgb, person_detections, obstacle_info, nav_cmd)
            self.publish_commands(nav_cmd)
```

### 5.2 待实现模块

- [ ] `unified_system.py` - 主集成文件
- [ ] `decision_maker.py` - 融合决策器
- [ ] `unified_visualizer.py` - 统一可视化界面
- [ ] `config_manager.py` - 配置管理器
- [ ] `performance_monitor.py` - 性能监控
- [ ] `logger.py` - 日志系统

---

## 6. 文件清理与整理建议

### 6.1 已废弃的文件 (可删除)
- `person_detect.py.bak`
- `person_detect_pytorch.py.bak`
- `HawkColor/` - 仅色彩（无深度）
- `HawkIr/` - 红外（单独）
- `HawkLightIr/` - 轻红外（单独）

### 6.2 文件重组建议

```
ros2-robt/
├─ README.md (新建综合说明)
├─ CODE_STRUCTURE.md (本文档)
├─ requirements.txt (现有)
├─ setup.py (现有)
│
├─ src/
│   ├─ core/
│   │   ├─ unified_system.py (新)
│   │   ├─ decision_maker.py (新)
│   │   ├─ config_manager.py (新)
│   │   └─ logger.py (新)
│   │
│   ├─ sensors/
│   │   ├─ berxel_camera.py (现有)
│   │   ├─ berxel_wrapper.cpp (现有)
│   │   └─ mid70_interface.py (新)
│   │
│   ├─ detection/
│   │   ├─ yolo_detector.py (现有)
│   │   └─ person_detect_slam.py (现有)
│   │
│   ├─ slam/
│   │   └─ depth_slam_obstacle.py (现有)
│   │
│   └─ visualization/
│       ├─ unified_visualizer.py (新)
│       └─ mid70_vis.py (现有)
│
├─ tests/
│   ├─ test_slam_module.py (移动)
│   ├─ test_berxel_camera.py (移动)
│   ├─ test_yolo.py (移动)
│   └─ test_*.py (其他移动)
│
├─ tools/
│   ├─ export_to_onnx.py (现有)
│   ├─ benchmark_engines.py (现有)
│   └─ ...
│
├─ ros2_ws/
│   └─ (现有)
│
└─ docs/
    ├─ ARCHITECTURE.md (新)
    ├─ PERFORMANCE.md (现有)
    └─ API_REFERENCE.md (新)
```

---

## 7. 编译与部署

### 7.1 环境要求
```bash
# Linux (推荐 Ubuntu 22.04 + ROS2 Humble)
Python 3.10+
GCC 11+
CUDA 11.8+ (可选，不推荐用AMD 780M)
ROCm 5.7+ (推荐用AMD 780M)
ROS2 Humble
```

### 7.2 快速启动
```bash
# 1. 编译C++模块
python setup.py build_ext --inplace

# 2. 安装依赖
pip install -r requirements.txt
pip install open3d

# 3. 运行主程序
python3 person_detect_slam.py

# 或启动ROS2系统
source /opt/ros/humble/setup.bash
ros2 launch ros2_ws launch_sensors.py
```

---

## 8. 关键配置参数

### 8.1 相机配置
```python
BERXEL_FPS = 30
BERXEL_RESOLUTION = (640, 480)
BERXEL_DEPTH_RANGE = (0.1, 5.0)  # 米
```

### 8.2 YOLO检测配置
```python
YOLO_MODEL = 'yolo12n.onnx'
YOLO_INPUT_SIZE = 416
YOLO_CONF_THRESHOLD = 0.40
YOLO_IOU_THRESHOLD = 0.45
```

### 8.3 SLAM配置
```python
DEPTH_NEAR = 0.5  # 米
DEPTH_FAR = 5.0   # 米
MIN_NAVIGABLE_AREA = 1000  # 像素
```

### 8.4 性能配置
```python
SKIP_FRAMES = 1
DISPLAY_SCALE = 0.5
TARGET_FPS = 30
ENABLE_GPU = True
```

---

## 9. 故障排查

| 问题 | 原因 | 解决方案 |
|------|------|---------|
| 相机初始化失败 | SDK路径错误 | 检查setup.py中SDK路径 |
| ONNX推理错误 | 模型格式不兼容 | 重新导出模型 |
| SLAM处理缓慢 | CPU不足 | 降低分辨率或跳帧 |
| 点云显示不出 | ROS2话题错误 | 检查激光雷达驱动是否运行 |
| GPU未被使用 | 驱动问题 | 检查ROCm/CUDA安装 |

---

## 10. 后续开发计划

- [ ] 多目标跟踪 (ByteTrack集成)
- [ ] ROS2导航栈集成 (Nav2)
- [ ] Web可视化界面
- [ ] 数据记录与回放
- [ ] 远程监控功能
- [ ] 模型量化与压缩
- [ ] 边缘设备部署 (Jetson)

---

**文档维护**: 每次添加新模块时更新此文档  
**联系方式**: Zhuo RM Team
