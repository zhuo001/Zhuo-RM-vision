# SLAM集成说明

## 📦 新增模块

### 1. depth_slam_obstacle.py
深度SLAM障碍物检测核心模块

**功能:**
- 实时深度图处理与去噪
- 障碍物检测与分割
- 可导航区域分析
- 导航方向建议

**使用方法:**
```python
from depth_slam_obstacle import DepthSLAMObstacleDetector

# 初始化
slam = DepthSLAMObstacleDetector(
    depth_threshold_near=0.8,  # 近距离障碍物阈值(米)
    depth_threshold_far=5.0,   # 远距离阈值(米)
    min_navigable_area=800     # 最小可导航区域(像素)
)

# 处理深度帧
obstacle_mask, info = slam.process_depth_frame(depth_meters)

# 可视化
vis = slam.visualize(depth_meters, obstacle_mask, info)
```

### 2. person_detect_slam.py
人员检测 + SLAM导航集成系统

**功能:**
- YOLOv8人员检测
- 深度SLAM避障
- 实时导航决策
- 统一可视化界面

**运行方法:**
```bash
# 激活虚拟环境
source .venv/bin/activate

# 运行集成系统
python person_detect_slam.py
```

**控制键:**
- `q`: 退出程序
- `s`: 保存截图
- `p`: 暂停/继续
- `d`: 切换SLAM显示开关

## 🎯 SLAM决策输出

决策系统输出包含以下信息:

```python
{
    'suggested_direction': 'forward',  # forward/left/right/stop
    'navigable_zones': [               # 可导航区域列表
        {
            'centroid': (x, y),        # 中心点坐标
            'area': 12345,             # 区域面积
            'bbox': (x, y, w, h),      # 边界框
            'score': 0.85              # 导航评分(0-1)
        },
        ...
    ],
    'obstacle_count': 42,              # 障碍物数量
    'min_depth': 1.23,                 # 最近障碍物距离(米)
    'processing_time': 0.032,          # 处理时间(秒)
    'frame_count': 1234                # 帧计数
}
```

## 🔧 参数调整

在 `person_detect_slam.py` 中可调整以下参数:

### SLAM参数
```python
SLAM_DEPTH_THRESHOLD_NEAR = 0.8  # 近距离障碍物阈值(米)
SLAM_DEPTH_THRESHOLD_FAR = 5.0   # 远距离阈值(米)

slam = DepthSLAMObstacleDetector(
    depth_threshold_near=0.8,        # 调整障碍物检测敏感度
    depth_threshold_far=5.0,         # 调整检测范围
    obstacle_height_min=0.1,         # 最小障碍物高度
    grid_resolution=0.05,            # 占据栅格分辨率
    min_navigable_area=800           # 最小可导航区域面积
)
```

### 性能参数
```python
DEPTH_PROCESS_INTERVAL = 3   # 每N帧处理一次深度图
SKIP_FRAMES = 1              # YOLO检测跳帧
YOLO_INPUT_SIZE = 416        # YOLO输入分辨率
TARGET_FPS = 30              # 目标帧率
```

## 📊 性能基准

在AMD 780M平台上测试结果:

| 模块 | 处理时间 | FPS |
|------|---------|-----|
| 相机采集 | ~5ms | - |
| YOLO检测 | ~25ms | 40 |
| SLAM处理 | ~30ms | 33 |
| 深度渲染 | ~10ms | - |
| **总计** | ~70ms | **28-30** |

## 🗺️ 与ROS2集成

可以将SLAM决策发布为ROS2消息:

```python
# 在ros2_ws/src/person_detector/person_detector/节点中
import rclpy
from geometry_msgs.msg import Twist

# 创建发布器
cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

# 根据SLAM决策发布速度命令
twist = Twist()
if direction == 'forward':
    twist.linear.x = 0.5
elif direction == 'left':
    twist.angular.z = 0.5
elif direction == 'right':
    twist.angular.z = -0.5
else:  # stop
    twist.linear.x = 0.0
    twist.angular.z = 0.0

cmd_vel_pub.publish(twist)
```

## 🔗 文件关系

```
ros2-robt/
├── person_detect.py              # 原始人员检测(已修复闪烁)
├── person_detect_slam.py         # 新：集成SLAM的检测系统 ⭐
├── depth_slam_obstacle.py        # 新：SLAM核心算法 ⭐
├── berxel_camera.py              # 相机接口
└── yolov8n.onnx                  # YOLO模型
```

## 🚀 快速开始

### 1. 测试SLAM模块
```bash
python -c "from depth_slam_obstacle import DepthSLAMObstacleDetector; print('✅ SLAM模块导入成功')"
```

### 2. 运行集成系统
```bash
python person_detect_slam.py
```

### 3. 查看效果
- 左侧: 人员检测结果
- 右侧: SLAM导航可视化
  - 红色区域: 障碍物
  - 绿色框: 可导航区域
  - 黄色箭头: 建议方向
  - 文字显示: 决策信息

## 📝 下一步开发

- [ ] ROS2节点封装
- [ ] 占据栅格地图持久化
- [ ] 多目标跟踪集成
- [ ] 路径规划算法(A*/DWA)
- [ ] 决策策略优化
- [ ] 性能进一步优化

## 🐛 故障排查

### 问题1: ImportError
```bash
# 确认模块在当前目录
ls depth_slam_obstacle.py

# 检查Python路径
python -c "import sys; print('\n'.join(sys.path))"
```

### 问题2: SLAM处理慢
```bash
# 调整处理间隔
DEPTH_PROCESS_INTERVAL = 5  # 增大间隔

# 或降低最小区域阈值
min_navigable_area=500  # 减少计算量
```

### 问题3: 方向建议不准确
```bash
# 调整障碍物阈值
SLAM_DEPTH_THRESHOLD_NEAR = 1.0  # 增大近距离阈值

# 或调整决策逻辑
# 在 _suggest_direction() 方法中修改判断条件
```

---

**更新日期**: 2025-10-17  
**作者**: Zhuo RM Team
