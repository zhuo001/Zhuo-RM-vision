# Person Detector ROS2 Package

基于 Berxel P100R 深度相机 + YOLOv8 + ByteTrack 的人体检测与跟踪系统

## 功能特性

### ✅ 已实现功能

1. **ROS2 集成**
   - 自定义消息类型（PersonDetection, PersonDetectionArray）
   - 标准 ROS2 节点架构
   - 话题发布/订阅机制

2. **YOLOv8 人体检测**
   - 高精度人体识别（YOLOv8n模型）
   - 智能人形验证（宽高比、面积、尺寸限制）
   - 置信度阈值过滤（默认0.55）
   - 支持GPU加速

3. **Berxel P100R 深度测距**
   - 彩色图：1920x1080 @ 30fps
   - 深度图：640x400 @ 30fps
   - 精确深度转换（1/17mm单位）
   - 7x7窗口中值滤波降噪
   - 有效距离：0.3m - 8m

4. **ByteTrack 多目标跟踪**
   - 唯一ID分配
   - 跨帧目标关联
   - 丢失目标重识别
   - 可配置跟踪参数

5. **性能优化**
   - GPU加速推理
   - 多线程并行处理
   - 相机采集与检测解耦
   - 实时性能监控（FPS统计）

## 系统架构

```
┌─────────────────┐
│  Berxel P100R   │
│  Depth Camera   │
└────────┬────────┘
         │ Color + Depth
         ▼
┌─────────────────────────────┐
│  Camera Capture Thread      │
│  (30 FPS)                   │
└────────┬────────────────────┘
         │ Frame Queue
         ▼
┌─────────────────────────────┐
│  YOLO Detection Threads     │
│  (GPU Accelerated)          │
└────────┬────────────────────┘
         │ Detection Results
         ▼
┌─────────────────────────────┐
│  ByteTrack Tracker          │
│  (Multi-Object Tracking)    │
└────────┬────────────────────┘
         │ Tracked Detections
         ▼
┌─────────────────────────────┐
│  ROS2 Publisher             │
│  /person_detections         │
│  /detection_debug_image     │
└─────────────────────────────┘
```

## 安装依赖

### 系统依赖

```bash
# ROS2 Humble (Ubuntu 22.04)
sudo apt install ros-humble-cv-bridge ros-humble-vision-msgs

# Python 依赖
pip install ultralytics opencv-python numpy lap scipy
```

### 编译包

```bash
cd ~/ros2_ws
colcon build --packages-select person_detector_msgs person_detector --symlink-install
source install/setup.bash
```

## 使用方法

### 1. 基础检测节点（无跟踪）

```bash
ros2 run person_detector person_detector_node
```

### 2. 跟踪节点（启用 ByteTrack）

```bash
ros2 run person_detector person_detector_node --ros-args -p use_tracking:=true
```

### 3. 高性能节点（GPU + 多线程 + 跟踪）

```bash
ros2 run person_detector performance_detector_node
```

### 4. 使用 Launch 文件

```bash
# 基础启动
ros2 launch person_detector person_detector.launch.py

# 启用跟踪
ros2 launch person_detector person_detector.launch.py use_tracking:=true

# 自定义参数
ros2 launch person_detector person_detector.launch.py \
    model_path:=yolov8n.pt \
    conf_threshold:=0.55 \
    use_tracking:=true \
    publish_debug_image:=true
```

## 话题说明

### 发布话题

#### `/person_detections` (person_detector_msgs/PersonDetectionArray)

多个人体检测结果数组

```yaml
header:
  stamp: <时间戳>
  frame_id: "berxel_camera_link"
detections:
  - header: <同上>
    track_id: 1              # 跟踪ID（-1表示未启用跟踪）
    x1: 245                  # 边界框左上角X
    y1: 120                  # 边界框左上角Y
    x2: 456                  # 边界框右下角X
    y2: 789                  # 边界框右下角Y
    center_x: 350            # 中心点X
    center_y: 454            # 中心点Y
    width: 211               # 宽度
    height: 669              # 高度
    confidence: 0.87         # 检测置信度
    distance: 1.25           # 距离（米）
    has_depth: true          # 是否有有效深度
count: 1                     # 检测总数
```

#### `/detection_debug_image` (sensor_msgs/Image)

调试可视化图像（带检测框和标注）

## 参数配置

### 通用参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `model_path` | string | yolov8n.pt | YOLO模型路径 |
| `conf_threshold` | float | 0.55 | 检测置信度阈值 |
| `iou_threshold` | float | 0.45 | NMS IOU阈值 |
| `publish_debug_image` | bool | true | 是否发布调试图像 |
| `use_tracking` | bool | false | 是否启用跟踪 |

### 性能节点额外参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `use_gpu` | bool | true | 是否使用GPU加速 |
| `num_threads` | int | 2 | 检测处理线程数 |

## 性能指标

### 测试环境
- CPU: Intel NUC
- GPU: NVIDIA (CUDA支持)
- 相机: Berxel P100R (1920x1080 @ 30fps)

### 性能表现

| 配置 | 延迟 | 吞吐量 | 备注 |
|------|------|--------|------|
| 基础节点（CPU） | ~50ms | ~20 FPS | 单线程 |
| 基础节点（GPU） | ~35ms | ~28 FPS | GPU加速 |
| 性能节点（GPU+多线程） | ~25ms | ~35+ FPS | 推荐配置 |

## 消息类型详解

### PersonDetection.msg

单个人体检测结果

```
std_msgs/Header header
int32 track_id            # 跟踪ID（-1=未跟踪）
int32 x1, y1, x2, y2      # 边界框坐标
int32 center_x, center_y  # 中心点
float32 confidence        # 置信度 [0.0, 1.0]
float32 distance          # 距离（米）
int32 width, height       # 尺寸
bool has_depth            # 深度有效性
```

### PersonDetectionArray.msg

多个检测结果数组

```
std_msgs/Header header
PersonDetection[] detections
int32 count
```

## 调试与监控

### 查看话题

```bash
# 列出所有话题
ros2 topic list

# 查看检测结果
ros2 topic echo /person_detections

# 查看话题频率
ros2 topic hz /person_detections
```

### 可视化调试图像

```bash
# 使用 rqt_image_view
ros2 run rqt_image_view rqt_image_view

# 或使用 rviz2
rviz2
```

## 常见问题

### Q1: 相机初始化失败

**错误**: `uvc_open ret = -6`

**解决**:
```bash
# 检查设备连接
lsusb | grep Berxel

# 重新插拔USB
# 或重启相机
```

### Q2: GPU不可用

**错误**: `CUDA not available`

**解决**:
```bash
# 检查CUDA安装
nvidia-smi

# 安装PyTorch CUDA版本
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu118

# 禁用GPU运行
ros2 run person_detector performance_detector_node --ros-args -p use_gpu:=false
```

### Q3: 检测精度不足

**调整参数**:
- 降低 `conf_threshold` (0.45 - 0.50)
- 检查光照条件
- 确认目标距离在有效范围内（0.3m - 8m）

### Q4: FPS过低

**优化建议**:
1. 启用GPU加速（`use_gpu:=true`）
2. 使用性能节点（`performance_detector_node`）
3. 增加处理线程数（`num_threads:=3`）
4. 关闭调试图像（`publish_debug_image:=false`）

## 进阶配置

### 自定义检测验证

修改 `is_valid_person()` 函数中的参数：

```python
# 宽高比范围
MIN_ASPECT_RATIO = 0.5
MAX_ASPECT_RATIO = 5.0

# 面积比例
MIN_AREA_RATIO = 0.005  # 0.5%
MAX_AREA_RATIO = 0.95   # 95%

# 绝对尺寸
MIN_WIDTH = 60
MIN_HEIGHT = 100
MAX_WIDTH = 1850
MAX_HEIGHT = 1070
```

### ByteTrack 跟踪参数

```python
tracker = ByteTracker(
    track_thresh=0.5,      # 高置信度跟踪阈值
    track_buffer=30,       # 最大丢失帧数
    match_thresh=0.8       # IOU匹配阈值
)
```

## 开发者指南

### 代码结构

```
person_detector/
├── person_detector/
│   ├── __init__.py
│   ├── detector_node.py              # 原始节点
│   ├── person_detector_node.py       # 基础ROS2节点
│   ├── performance_detector_node.py  # 高性能节点
│   └── byte_tracker.py               # ByteTrack实现
├── launch/
│   └── person_detector.launch.py     # Launch文件
├── package.xml
└── setup.py
```

### 添加新功能

1. 在 `person_detector/` 目录下创建新模块
2. 在 `setup.py` 中注册新的可执行文件
3. 更新 `package.xml` 添加依赖
4. 重新编译包

## 许可证

MIT License

## 维护者

- **作者**: zhuo-skadi
- **邮箱**: zhuo@example.com
- **仓库**: https://github.com/zhuo001/Zhuo-RM-vision

## 更新日志

### v0.3.0 (2025-01-08)
- ✅ 添加高性能多线程节点
- ✅ 实现GPU加速推理
- ✅ 集成ByteTrack多目标跟踪
- ✅ 添加FPS性能监控

### v0.2.0 (2025-01-07)
- ✅ 创建自定义消息类型
- ✅ 实现基础ROS2节点
- ✅ 添加深度测距功能

### v0.1.0 (2025-01-06)
- ✅ Berxel P100R相机集成
- ✅ YOLOv8人体检测
- ✅ 深度单位校准（1/17mm）
