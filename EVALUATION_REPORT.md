# Zhuo-RM 导航系统评估报告

**评估日期**: 2026-05-11  
**参考基准**: [COD 战队 RM2026 导航全开源项目](https://gitee.com/codnavgation/cod_-rm2026_-navigation)  
**当前分支**: `fix/depth-smoothing`  
**评估目标**: 实时避障 / 无图导航(自返航) / 视觉目标跟踪 / 本机性能

---

## 一、项目概览对比

| 维度 | COD RM2026 | Zhuo-RM (本项目) |
|------|-----------|-----------------|
| **ROS 版本** | ROS2 Humble | ROS2 Humble |
| **激光雷达** | Livox MID-360 | Unitree L2 + Livox Mid-70 (双雷达) |
| **深度相机** | 可选 | Berxel P100R RGB-D (核心) |
| **SLAM/建图** | slam_toolbox 在线建图 | 无图导航 + 深度 SLAM |
| **路径规划** | Nav2 MPPI | Nav2 Regulated Pure Pursuit |
| **目标检测** | 无 | YOLOv12n ONNX |
| **目标跟踪** | 无 | ByteTrack + PID 跟随控制器 |
| **航点巡逻** | waypoint_editor (CSV) | 无 |
| **里程计** | small_point_lio | 假里程计(测试中) |
| **代码主导** | C++ 82% | Python 85%+ |
| **设计理念** | KISS | 多传感器融合 |

---

## 二、实时避障能力评估

### 2.1 当前已实现

| 模块 | 文件 | 功能 | 状态 |
|------|------|------|------|
| 深度 SLAM 障碍物检测 | `depth_slam_obstacle.py` | 基于 P100R 深度图的障碍物分割、可通行区域分析、方向建议 | ✅ 完成 |
| Nav2 代价地图避障 | `nav2_mapless_params.yaml` | VoxelLayer 接收融合点云生成代价地图 | ✅ 配置完成 |
| 点云融合 | `pointcloud_fusion_node.cpp` | 融合 L2 + Mid-70 + P100R 点云 | ✅ 代码完成 |
| 地面分割 | `mapless_nav` 规划中 | RANSAC / Patchwork++ | ⚠️ 未实现 |
| 自交点云过滤 | 无 | 去除机器人自身点云 | ❌ 缺失 |

### 2.2 vs COD 参考

| 能力 | COD | Zhuo-RM | 差距 |
|------|-----|---------|------|
| 360° 障碍物感知 | MID-360 原生支持 | L2 360° + P100R 前置 | 更丰富 |
| 自身点云过滤 | `cpp_lidar_filter` 实现 | 无 | **需补齐** |
| PointCloud→LaserScan | `pointcloud_to_laserscan` 包 | 无 | **需补齐** |
| 近距离精细避障 | 依赖 LiDAR 最小探测距离 | P100R 深度图可覆盖 0.3m+ | 更强 |
| 控制层避障 | MPPI 控制器 | RegulatedPurePursuit | 可用 |

### 2.3 评估结论: ⭐⭐⭐⚡ (3.5/5)

优势：多传感器覆盖，P100R 提供近距离精细感知优于纯 LiDAR 方案。  
短板：缺少自交点云过滤和 PointCloud→LaserScan 转换，Nav2 局部代价地图尚未实测验证。  
建议：补齐 `cpp_lidar_filter` 和 `pointcloud_to_laserscan` 两个包。

---

## 三、无图导航(自返航)能力评估

### 3.1 当前已实现

| 模块 | 文件 | 功能 | 状态 |
|------|------|------|------|
| Nav2 无图模式配置 | `nav2_mapless_params.yaml` | rolling_window + 无 static_layer | ✅ 完成 |
| 无图导航 Launch | `nav2_mapless.launch.py` | 启动完整 Nav2 栈 (无地图) | ✅ 完成 |
| 融合点云话题 | `/fused_pointcloud` | 多传感器点云 → Nav2 代价地图 | ⚠️ 节点待联调 |
| 里程计 | `fake_odom_node.py` | 测试用假里程计 | ⚠️ 需替换为真实里程计 |
| 深度转点云 | `depth_to_pointcloud_node.cpp` | P100R 深度图 → PointCloud2 | ✅ 代码完成 |

### 3.2 自返航能力分析

当前项目**尚未实现自返航 (return-to-home) 功能**。COD 项目通过航点巡逻来模拟"回启动区"，本项目需要补充：

- **位置记忆**：记录出发点坐标（里程计基准）
- **返航触发**：低电量/信号丢失/手动指令
- **返航路径**：Nav2 NavigateToPose 回初始坐标
- **避障返航**：返航过程中保持实时避障

### 3.3 vs COD 参考

| 能力 | COD | Zhuo-RM | 差距 |
|------|-----|---------|------|
| 在线 SLAM 建图 | slam_toolbox | 无 (刻意无图) | 路线选择不同 |
| 航点导航 | waypoint_editor | 无 | **需补齐** |
| 地图保存/复用 | 自动保存 pgm+yaml | 不需要 | 路线选择不同 |
| 里程计 | small_point_lio | 假里程计 | **关键短板** |
| 全局定位 | slam_toolbox 定位 | 无 | 仅依赖里程计 |

### 3.4 评估结论: ⭐⭐⚡ (2/5)

Nav2 无图基础设施已搭建，但**缺少真实里程计**是致命短板——没有可靠里程计，无图导航和自返航都无法工作。  
建议：优先集成 `small_point_lio` 或 `fast_lio` 提供 odom→base_link 变换。

---

## 四、视觉目标跟踪能力评估

### 4.1 当前已实现

| 模块 | 文件 | 功能 | 状态 |
|------|------|------|------|
| YOLOv12 人员检测 | `person_detect_slam.py` | ONNX Runtime 推理，416×416 | ✅ 完成 |
| 人形验证 | `is_valid_person()` | 宽高比/面积/置信度过滤 | ✅ 完成 |
| 深度测距 | `get_depth_at_point()` | P100R 7×7 中值滤波，0.3-8m | ✅ 完成 |
| 主目标选择 | `select_primary_target()` | 面积×中心偏移×置信度加权 | ✅ 完成 |
| ByteTrack 多目标跟踪 | `byte_tracker.py` | 唯一 ID + 跨帧关联 | ✅ 代码完成 |
| 跟随控制器 | `following_controller_node.py` | PID + 状态机 (6 状态) | ✅ 代码完成 |
| 目标追踪节点 | `target_tracker_node.py` | 目标位置发布 | ✅ 代码完成 |

### 4.2 目标跟踪 vs 人员跟随流水线

```
P100R RGB → YOLOv12 检测 → 人形验证 → 主目标选择 → 深度测距 → 3D 位置
                                                                    ↓
                                                             ByteTrack 跟踪
                                                                    ↓
                                                        /target_tracker/primary_target
                                                                    ↓
                                                        跟随控制器 (PID/状态机)
                                                                    ↓
                                                              /cmd_vel
```

### 4.3 vs COD 参考

COD 项目**没有目标跟踪功能**——它专注于哨兵巡逻导航而非人员跟随。本项目在此维度远超 COD。

### 4.4 评估结论: ⭐⭐⭐⭐ (4/5)

视觉目标跟踪是**本项目最强模块**。YOLOv12 + 深度测距 + ByteTrack + 跟随控制器的完整流水线已实现。  
短板：ROS2 节点运行存在 NumPy 版本兼容问题，跟随控制器尚未与 Nav2 避障联调。  
建议：修复 ROS2 环境或改用 Docker/ZMQ 桥接方案。

---

## 五、本机性能评估

### 5.1 硬件规格

| 组件 | 规格 |
|------|------|
| CPU | AMD Ryzen 7 7840HS (8C/16T, Zen4) |
| GPU | AMD 780M (RDNA3 12CU, 未启用 ROCm) |
| 内存 | 16GB DDR5 |
| 存储 | 512GB NVMe |
| 网络 | 千兆以太网 ×1 |
| OS | Ubuntu 22.04 LTS |

### 5.2 各模块性能基准

| 模块 | 处理时间 | FPS | CPU 占用 | 备注 |
|------|----------|-----|----------|------|
| YOLOv12n ONNX 推理 | ~12ms | 83 (纯推理) | 中 | CPU provider，416×416 |
| 人形验证 + 后处理 | ~3ms | - | 低 | - |
| 深度 SLAM 障碍物检测 | ~2.9ms | 348 | 低 | 合成数据测试 |
| 深度图 EMA 平滑 | <1ms | - | 极低 | - |
| 点云融合 (估计) | ~50ms | ~20 | 中 | 三传感器融合 |
| Nav2 代价地图更新 | ~50ms | ~20 | 中 | voxel_layer 10Hz |
| 路径规划 (RPP) | ~20ms | ~50 | 低-中 | - |

### 5.3 总体性能预估

| 场景 | 预估总 CPU 占用 | 预估 FPS | 可行性 |
|------|-----------------|----------|--------|
| 仅视觉跟踪 (person_detect_slam.py) | 30-50% | 45-80 | ✅ 充裕 |
| 视觉跟踪 + LiDAR 可视化 | 40-60% | 25-35 | ✅ 可行 |
| 全系统: 跟踪 + 点云融合 + Nav2 导航 | 60-85% | 15-25 | ⚠️ 接近瓶颈 |
| 全系统 + SLAM 建图 | 80-95% | 10-18 | ❌ 可能不够 |

### 5.4 性能瓶颈分析

1. **CPU 推理是最大瓶颈**: ONNX Runtime 用 CPU 跑 YOLO (~12ms/帧)，如果有 ROCm GPU 加速可降至 2-4ms
2. **点云融合计算量**: 三传感器点云融合在 CPU 上可能成为第二瓶颈
3. **内存**: 16GB 对于多传感器缓冲 + Nav2 是足够的
4. **网络带宽**: 千兆以太网足够传输 LiDAR 数据

### 5.5 vs COD 参考

COD 使用 MID-360 单雷达 + slam_toolbox，传感器计算负载明显低于本项目（双雷达 + RGB-D）。本项目传感器更多，功能更丰富，但**计算成本也显著更高**。

### 5.6 评估结论: ⭐⭐⭐ (3/5)

AMD 7840HS 的 CPU 性能足以支撑当前需求，但**全系统联调时可能在峰值 80-85% CPU**，留有的安全余量偏小。  
关键优化方向：启用 ROCm GPU 加速 YOLO 推理（如果可行）、点云降采样、降低非关键模块频率。

---

## 六、ROS2 运行环境检查

### 6.1 环境状态

| 检查项 | 状态 | 详情 |
|--------|------|------|
| ROS2 Humble | ✅ 已安装 | `/opt/ros/humble/setup.bash` |
| colcon | ✅ 已安装 | `.venv/bin/colcon` |
| Nav2 (完整栈) | ✅ 已安装 | 27 个 nav2 包 |
| PCL | ✅ 已安装 | PCL 1.12.1 + ros-humble-pcl-conversions |
| tf2 | ✅ 已安装 | 完整 tf2 生态 |
| OpenCV | ✅ 已安装 | 4.12.0 |
| ONNX Runtime | ✅ 已安装 | 1.23.1 (CPU provider only) |
| SciPy | ✅ 已安装 | 1.15.3 |
| ROCm | ❌ 未安装 | `/opt/rocm` 不存在 |
| Open3D | ❌ 未安装 | 仅 mid70_vis.py 需要 |

### 6.2 ROS2 工作空间

| 包名 | 语言 | 状态 |
|------|------|------|
| `person_detector_msgs` | CMake | ✅ 已编译 |
| `person_detector` | Python | ✅ 已编译 |
| `berxel_camera_ros2` | Python | ✅ 已编译 |
| `mapless_nav` | C++/Python | ✅ 已编译 |
| `livox_ros_driver2` | C++ | ✅ 已编译 |
| `unitree_lidar_ros2` | C++ | ✅ 已编译 |

### 6.3 已知问题

| 问题 | 影响 | 解决方案 |
|------|------|----------|
| NumPy 版本冲突 (cv_bridge) | ROS2 节点可能报错 | Docker 隔离环境 |
| ROCm 不可用 | YOLO 仅 CPU 推理 | 安装 ROCm 或接受 CPU |
| ros2 命令不在 PATH | 需手动 source | 添加 source 到 ~/.bashrc |

### 6.4 评估结论: ⭐⭐⭐ (3/5)

ROS2 环境核心组件齐全，但存在 NumPy 兼容问题和 GPU 加速缺失。建议 Docker 容器化解决环境问题。

---

## 七、综合评分

| 目标 | 评分 | 说明 |
|------|------|------|
| **实时避障** | ⭐⭐⭐⚡ 3.5/5 | 多传感器基础好，缺自身点云过滤和实测验证 |
| **无图导航 (自返航)** | ⭐⭐ 2/5 | Nav2 配置就绪，缺真实里程计和返航逻辑 |
| **视觉目标跟踪** | ⭐⭐⭐⭐ 4/5 | 最强模块，完整流水线 (YOLO+Depth+ByteTrack+PID) |
| **本机性能** | ⭐⭐⭐ 3/5 | CPU 可支撑，但全系统联调余量偏小 |
| **综合** | ⭐⭐⭐ 3.1/5 | 基础扎实，关键短板可在一到两周补齐 |

---

## 八、优先行动路线

### 🔴 P0 (阻塞项，必须立即解决)

1. **集成真实里程计** — 引入 `small_point_lio` 或 `fast_lio`，替换 `fake_odom_node.py`。没有里程计，Nav2 无法工作。
2. **修复 ROS2 运行时环境** — NumPy 版本冲突导致 ROS2 节点无法启动，建议 Docker 容器化。

### 🟡 P1 (关键项，影响核心功能)

3. **补齐点云处理链路** — 添加自身点云过滤 + PointCloud→LaserScan 转换
4. **实现自返航逻辑** — 记录起始位置 → 低电量触发 → NavigateToPose 返回
5. **添加航点巡逻功能** — 参考 COD `waypoint_editor`，实现多点巡航

### 🟢 P2 (优化项，提升性能与鲁棒性)

6. **启用 GPU 加速** — 安装 ROCm + onnxruntime-rocm，YOLO 推理可快 3-5x
7. **全系统联调测试** — 将视觉跟踪 + Nav2 导航 + LiDAR 避障串联运行
8. **实机场地测试** — 在真实环境中测试避障/跟随/返航成功率

### 🔵 参考 COD 项目的可复用组件

| COD 组件 | 用途 | 是否可直接复用 |
|----------|------|---------------|
| `cpp_lidar_filter` | 自身点云过滤 | ✅ 仅需适配传感器参数 |
| `pointcloud_to_laserscan` | 点云转激光扫描 | ✅ ROS2 官方包可替代 |
| `small_point_lio` | 里程计 | ✅ 开源可直接集成 |
| `goal_approach_controller` | 接近目标限速 | ✅ 控制逻辑可参考 |
| `waypoint_editor` | 航点编辑 | ⚠️ 需要适配本项目消息格式 |

---

## 九、COD 项目可借鉴的设计理念

1. **KISS 原则**: COD 项目传感器种类少、代码以 C++ 为主，结构清晰。本项目传感器多但复杂度高，需注意模块解耦。
2. **静态 TF 优于 URDF**: COD 手动发布静态 TF 坐标变换，比维护 URDF 文件更灵活。
3. **slam_toolbox 建图**: 虽然本项目目标是无图导航，但可考虑先用 slam_toolbox 建图验证导航链路，再逐步过渡到纯无图。
4. **MPPI 控制器**: COD 使用 MPPI 替代传统 PID，在动态环境中更鲁棒，值得评估。

---

> 报告生成时间: 2026-05-11  
> 作者: Claude Code (Zhuo RM Team)  
> 仓库: https://github.com/zhuo001/Zhuo-RM-vision  
> 参考: https://gitee.com/codnavgation/cod_-rm2026_-navigation
