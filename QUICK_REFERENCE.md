# 🚀 快速参考 - 代码模块速查表

## 📦 核心模块一览

### 1️⃣ 相机驱动
| 文件 | 功能 | 入口 | 输出 |
|------|------|------|------|
| `berxel_camera.py` | RGB-D驱动封装 | `BerxelCamera()` | RGB帧 + 深度图 |
| `berxel_wrapper.cpp` | C++ SDK桥接 | - | .so库文件 |
| `test_berxel_camera.py` | 驱动测试 | `python3 test_berxel_camera.py` | 帧率/分辨率验证 |

### 2️⃣ 目标检测
| 文件 | 功能 | 入口 | 输出 |
|------|------|------|------|
| `person_detect.py` | ONNX YOLO推理 | `python3 person_detect.py` | 目标框 + 置信度 |
| `berxel_person_detect.py` | PyTorch YOLO推理 | `python3 berxel_person_detect.py` | 目标框 + 深度值 |
| `test_yolo.py` | 模型验证 | `python3 test_yolo.py` | 推理成功验证 |
| `tools/export_to_onnx.py` | 模型导出 | `python3 tools/export_to_onnx.py` | yolo12n.onnx |

### 3️⃣ SLAM & 障碍物
| 文件 | 功能 | 入口 | 输出 |
|------|------|------|------|
| `depth_slam_obstacle.py` | 深度分析器 | `DepthSLAMObstacleDetector()` | 障碍物掩码 + 导航建议 |
| `test_slam_module.py` | SLAM测试 | `python3 test_slam_module.py` | 可视化 + 性能指标 |
| `debug_pointcloud.py` | 点云诊断 | `ros2 run ros2_ws debug_pointcloud` | PointCloud2信息 |

### 4️⃣ 集成版本
| 文件 | 功能 | 入门 | 特点 |
|------|------|------|------|
| `person_detect_slam.py` | **主集成程序** | `python3 person_detect_slam.py` | 融合YOLO+SLAM |
| `mid70_vis.py` | 雷达可视化 | `python3 mid70_vis.py` | Open3D点云显示 |

---

## ⚙️ 启动命令速查

### 快速启动（无ROS2）
```bash
# 1. 编译C++ Wrapper
cd /home/zhuo-skadi/Documents/ros2-robt
python setup.py build_ext --inplace

# 2. 启动主程序
python3 person_detect_slam.py

# 3. 启动激光雷达可视化（需单独运行）
python3 mid70_vis.py
```

### ROS2启动流程
```bash
# 1. 激活ROS2环境
source /opt/ros/humble/setup.bash

# 2. 启动传感器驱动
ros2 launch ros2_ws/launch_sensors.py

# 3. 在另一终端启动可视化节点
ros2 run ros2_ws mid70_vis.py

# 4. 运行主程序（可选，如果要独立运行）
python3 person_detect_slam.py
```

### 测试与诊断
```bash
# 测试相机
python3 test_berxel_camera.py

# 测试YOLO模型
python3 test_yolo.py

# 测试SLAM模块
python3 test_slam_module.py

# 诊断点云话题
python3 debug_pointcloud.py

# 性能基准测试
python3 test_onnx_performance.py
```

---

## 🎛️ 关键参数调整

### 🎯 YOLO检测参数
**文件**: `person_detect.py` / `person_detect_slam.py` 第50-60行
```python
YOLO_CONF_THRESHOLD = 0.40  # ↑ 增加=更严格，↓ 降低=更宽松
YOLO_IOU_THRESHOLD = 0.45   # NMS去重阈值
YOLO_INPUT_SIZE = 416       # 推理分辨率（降低=更快，质量↓）
```

### 🚧 SLAM检测参数
**文件**: `depth_slam_obstacle.py` 第20-40行
```python
depth_threshold_near = 0.5   # 近距离阈值(米)，更小=更敏感
depth_threshold_far = 5.0    # 远距离阈值(米)
min_navigable_area = 1000    # 最小可导航面积(像素)
```

### ⚡ 性能优化参数
**文件**: `person_detect_slam.py` 第60-70行
```python
SKIP_FRAMES = 1              # 跳帧数（1=每帧，2=每隔1帧）
DISPLAY_SCALE = 0.5          # 显示缩放比例(0-1)
TARGET_FPS = 30              # 目标帧率
DEPTH_PROCESS_INTERVAL = 3   # SLAM处理间隔
```

---

## 📊 性能基准值 (AMD 780M)

| 操作 | 耗时 | FPS | 备注 |
|------|------|------|------|
| 相机获取 | 5ms | 200 | RGB+Depth同步 |
| YOLO推理(416) | 18ms | 55 | ONNX CPU模式 |
| SLAM处理 | 12ms | 83 | 降采样处理 |
| 可视化渲染 | 8ms | 125 | OpenCV显示 |
| **总处理时间** | **35-40ms** | **25-28** | 整合版本 |

---

## 🔍 常见问题速查

### ❌ 相机初始化失败
```
原因: berxel_wrapper.so 未编译
解决: python setup.py build_ext --inplace
```

### ❌ ONNX运行时错误
```
原因: ONNX Runtime提供器不支持
解决: pip install onnxruntime  # 重新安装
      或手动指定CPU: providers = ['CPUExecutionProvider']
```

### ❌ SLAM处理缓慢
```
原因: DEPTH_PROCESS_INTERVAL 过小，或分辨率过高
解决: DEPTH_PROCESS_INTERVAL = 3  # 增加间隔
      或 降低深度图分辨率
```

### ❌ 点云话题无数据
```
原因: 激光雷达驱动未启动，或话题名称错误
解决: ros2 topic list  # 检查话题名称
      修改 mid70_vis.py 中的 topic_name
```

### ⚠️ GPU未被使用
```
原因: ROCm/CUDA驱动问题
诊断: python3 -c "import onnxruntime; print(onnxruntime.get_available_providers())"
解决: 安装ROCm (AMD): pip install onnxruntime-rocm
```

---

## 📂 文件查询

### 按功能查找
```
🎥 相机相关:
  └─ berxel_camera.py, berxel_wrapper.cpp, test_berxel_camera.py

🎯 检测相关:
  ├─ person_detect.py (ONNX)
  ├─ berxel_person_detect.py (PyTorch)
  ├─ test_yolo.py
  └─ tools/ (模型导出、基准测试)

🧭 SLAM相关:
  ├─ depth_slam_obstacle.py
  ├─ test_slam_module.py
  └─ debug_pointcloud.py

📡 激光雷达相关:
  ├─ mid70_vis.py (可视化)
  ├─ test_lidar_params.py (参数测试)
  └─ ros2_ws/ (ROS2驱动)

🔧 集成相关:
  ├─ person_detect_slam.py (主程序)
  ├─ CODE_STRUCTURE.md (本文档)
  └─ INTEGRATION_PLAN.md (整合计划)
```

### 按路径查找
```
根目录:
  ├─ person_detect*.py
  ├─ depth_slam_obstacle.py
  ├─ berxel_*.{py,cpp}
  └─ mid70_vis.py

Include/:
  └─ Berxel SDK头文件

Common/:
  └─ Berxel通用代码

HawkXXX/:
  └─ 各类传感器示例（可参考）

tools/:
  ├─ export_to_onnx.py
  ├─ benchmark_*.py
  └─ onnx_to_tensorrt.py

ros2_ws/:
  └─ ROS2工作空间

tests/ (待建):
  └─ 所有test_*.py文件
```

---

## 🛠️ 编译与构建

### 编译C++ Wrapper
```bash
python setup.py build_ext --inplace
# 输出: berxel_wrapper.*.so (在根目录)
```

### 构建ROS2工作空间
```bash
cd ros2_ws
colcon build
source install/setup.bash
```

### 安装依赖
```bash
pip install -r requirements.txt
pip install open3d  # 激光雷达可视化
```

---

## 📈 版本对应关系

| 功能 | 简单版 | 融合版 | 完整版(规划) |
|------|---------|---------|------------|
| 相机采集 | ✅ | ✅ | ✅ |
| YOLO检测 | ✅ | ✅ | ✅ |
| SLAM分析 | ❌ | ✅ | ✅ |
| 激光雷达 | ❌ | ❌ | ✅ |
| ROS2接口 | ❌ | 部分 | ✅ |
| 统一界面 | ❌ | ❌ | ✅ |
| 融合决策 | ❌ | ❌ | ✅ |

---

## 🎓 学习路径建议

### 初学者
1. 阅读 README.md (项目简介)
2. 运行 `test_berxel_camera.py` (验证硬件)
3. 运行 `test_yolo.py` (验证推理)
4. 运行 `person_detect.py` (简单检测)

### 中级用户
1. 学习 `person_detect_slam.py` (融合逻辑)
2. 调整参数进行优化
3. 运行 `test_slam_module.py` (理解SLAM)
4. 修改可视化代码

### 高级用户
1. 研究 `depth_slam_obstacle.py` (算法细节)
2. 参与整合版本开发
3. 优化性能与鲁棒性
4. 扩展新功能

---

## 📞 调试支持

### 启用详细日志
```python
# 在main程序开头添加
import logging
logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)
```

### 性能分析
```python
import cProfile
cProfile.run('main()')  # 生成性能报告
```

### 内存检查
```bash
python -m memory_profiler person_detect_slam.py
```

---

**快速参考卡版本**: 1.0  
**最后更新**: 2026-01-18  
**维护者**: Zhuo RM Team

💡 **提示**: 将此文件加入书签，方便快速查阅！
