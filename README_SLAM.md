# 深度图 SLAM 避障系统

## 概述

基于 Berxel 3D 相机的实时障碍物检测与路径规划系统，用于哨兵机器人的自主导航。

## 主要功能

- ✅ 实时深度图处理与滤波
- ✅ 障碍物检测与分割
- ✅ 可通行区域分析
- ✅ 简单的路径规划建议（前进/左转/右转/停止）
- ✅ 实时可视化显示
- 🚧 占据栅格地图（Occupancy Grid）- 待完善
- 🚧 3D点云可视化 - 待实现

## 快速开始

### 1. 环境准备

```powershell
# 创建并激活虚拟环境
cd Zhuo-RM-vision
python -m venv .venv
.\.venv\Scripts\Activate.ps1

# 安装依赖
python -m pip install --upgrade pip
python -m pip install numpy opencv-python
```

### 2. 运行测试（模拟数据）

```powershell
python depth_slam_obstacle.py
```

程序将使用模拟深度数据进行测试，显示：
- 深度图伪彩色可视化
- 障碍物检测掩码
- 可通行区域标记
- 移动方向建议

**快捷键：** 按 `q` 退出

### 3. 集成 Berxel 相机（待实现）

```python
from berxel_camera import BerxelCamera
from depth_slam_obstacle import DepthSLAMObstacleDetector

# 初始化相机
camera = BerxelCamera()
camera.open()

# 初始化检测器
detector = DepthSLAMObstacleDetector(
    depth_threshold_near=0.5,  # 50cm 内为近距离障碍
    depth_threshold_far=5.0,   # 5m 外忽略
    obstacle_height_min=0.1,   # 10cm 以上视为障碍
    grid_resolution=0.05       # 5cm 网格分辨率
)

while True:
    # 从相机读取深度图
    color_frame, depth_frame = camera.get_frames()
    
    # 处理深度帧
    obstacle_mask, info = detector.process_depth_frame(depth_frame, color_frame)
    
    # 获取导航建议
    direction = info['suggested_direction']  # 'forward', 'left', 'right', 'stop'
    
    # 可视化
    vis_image = detector.visualize(depth_frame, obstacle_mask, info, color_frame)
    cv2.imshow('SLAM Obstacle Detection', vis_image)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

camera.close()
cv2.destroyAllWindows()
```

## 核心算法

### 1. 深度图滤波
- 中值滤波去除椒盐噪声
- 双边滤波保留边缘细节
- 去除无效深度值（≤0 或 > 远距离阈值）

### 2. 障碍物检测
- **近距离检测**：深度值 < `depth_threshold_near` 的区域
- **边缘检测**：使用 Sobel 算子计算深度梯度，梯度大的区域视为障碍物边缘
- **形态学膨胀**：增加安全边界（15x15 椭圆核，2次膨胀）

### 3. 可通行区域分析
- 连通域分析找到所有可通行区域
- 按面积排序，选择最大的可通行区域
- 过滤小于阈值（1000像素）的区域

### 4. 路径规划建议
- 根据最大可通行区域的质心位置决定方向：
  - 质心在图像中心 ±20%：建议 **前进**
  - 质心在左侧：建议 **左转**
  - 质心在右侧：建议 **右转**
  - 无可通行区域：建议 **停止**

## 可调参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `depth_threshold_near` | 0.5m | 近距离阈值，小于此值的认为太近 |
| `depth_threshold_far` | 5.0m | 远距离阈值，大于此值的忽略 |
| `obstacle_height_min` | 0.1m | 最小障碍物高度 |
| `grid_resolution` | 0.05m | 占据栅格地图的分辨率（5cm） |
| 梯度阈值 | 0.5 | 深度梯度大于此值视为障碍物边缘 |
| 膨胀核大小 | 15x15 | 形态学膨胀的核大小 |
| 最小可通行面积 | 1000像素 | 小于此面积的可通行区域被忽略 |

## 性能指标

基于模拟数据测试（640x480 分辨率）：
- **处理速度**: ~30-50 FPS（CPU）
- **平均延迟**: ~20-30ms/帧

## 输出信息

```python
info = {
    'frame_count': int,              # 已处理帧数
    'processing_time': float,        # 当前帧处理时间（秒）
    'avg_processing_time': float,    # 平均处理时间（最近30帧）
    'obstacle_count': int,           # 检测到的障碍物像素数
    'navigable_zones': List[dict],   # 可通行区域列表
    'suggested_direction': str,      # 建议方向
    'min_depth': float,              # 最小深度值（米）
    'max_depth': float,              # 最大深度值（米）
}
```

## TODO 列表

- [ ] 实现占据栅格地图投影（需要相机内参）
- [ ] 添加 3D 点云可视化
- [ ] 集成 ROS2 节点发布导航命令
- [ ] 实现基于 A* 或 RRT 的全局路径规划
- [ ] 添加动态障碍物跟踪
- [ ] 优化算法性能（OpenCL/CUDA加速）
- [ ] 添加单元测试

## 依赖项

- Python 3.10+
- NumPy >= 2.2.6
- OpenCV >= 4.12.0

## 相关文件

- `depth_slam_obstacle.py` - 主程序
- `berxel_camera.py` - Berxel 相机接口（已有）
- `person_detect.py` - 人形检测系统（已有）

## 贡献

欢迎提交 Issue 和 Pull Request！

## 作者

Zhuo - 2025年10月15日

## 许可证

MIT License
