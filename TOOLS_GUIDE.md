# 🛠️ 便携式工具包使用指南

## 📦 工具概览

本项目提供了完整的便携式部署工具，解决"带出去用就出问题"的所有场景。

---

## 🚀 核心工具（3个必备）

### 1. `run.sh` - 一键启动 ⭐⭐⭐⭐⭐

**最重要的工具！90%的情况只需要这一个命令。**

```bash
./run.sh                           # 启动默认程序
./run.sh person_detect.py          # 启动指定程序
```

**功能:**
- ✓ 自动设置 LD_LIBRARY_PATH 和 PYTHONPATH
- ✓ 检查并修复 USB 权限
- ✓ 清理冲突进程
- ✓ 验证 Python 依赖
- ✓ 启动程序

**适用场景:**
- 每次启动程序
- 换到新环境
- 遇到权限问题
- 环境变量丢失

---

### 2. `portable_check.sh` - 环境诊断

**遇到问题时第一个运行的工具。**

```bash
./portable_check.sh
```

**检查项目（10大类）:**
1. 基础环境（OS、Python）
2. Python 依赖
3. 关键文件
4. USB 设备
5. 用户组权限
6. udev 规则
7. 动态库路径
8. 进程冲突
9. 硬件加速
10. 系统资源

**输出示例:**
```
✓ 环境完美！可以正常运行
或
✗ 发现 2 个严重问题 + 3 个警告
  （并给出修复建议）
```

**适用场景:**
- 程序无法启动
- 换到新机器
- 性能异常
- 定期检查

---

### 3. `fix_camera_permission.sh` - USB 权限修复

**专门解决摄像头权限问题。**

```bash
./fix_camera_permission.sh
```

**功能:**
- 检测 Berxel 摄像头
- 显示当前权限状态
- 提供5种修复方案
- 可选自动修复

**适用场景:**
- 错误码 -6
- 摄像头打开失败
- 换 USB 端口后

---

## 🔧 辅助工具（4个可选）

### 4. `save_environment.sh` - 环境快照

**保存当前工作配置，用于对比和恢复。**

```bash
./save_environment.sh
```

**生成内容:**
- 系统信息
- Python 环境
- 精确包版本 (requirements_exact.txt)
- ONNX Runtime 配置
- 硬件信息
- USB 设备
- 用户权限
- 环境变量
- 完整诊断报告

**生成文件:**
```
environment_snapshot_20251014_123456/
  ├── system_info.txt
  ├── python_env.txt
  ├── requirements_exact.txt
  ├── onnxruntime_info.txt
  ├── hardware_info.txt
  ├── usb_devices.txt
  ├── user_permissions.txt
  ├── environment_vars.txt
  └── diagnostic_report.txt

environment_snapshot_20251014_123456.tar.gz  (打包文件)
```

**使用场景:**
- 在工作环境保存配置
- 带到新机器对比差异
- 问题排查
- 配置备份

---

### 5. `demo_portable.sh` - 交互式演示

**展示便携式部署的完整流程。**

```bash
./demo_portable.sh
```

**演示内容:**
1. 运行环境诊断
2. 一键启动程序
3. 显示使用总结

**适用场景:**
- 学习工具使用
- 演示给团队
- 培训新成员

---

### 6. `install_accelerated_backends.sh` - 后端安装向导

**检测硬件并推荐最佳加速后端。**

```bash
./install_accelerated_backends.sh
```

**功能:**
- 自动检测 CPU/GPU 类型
- 推荐最佳后端（OpenVINO/ROCm/CUDA）
- 给出安装命令
- 性能预估

**适用场景:**
- 性能优化
- 安装加速后端
- 硬件升级后

---

### 7. `compare_versions.py` - 版本对比

**自动对比两个版本的性能差异。**

```bash
python3 compare_versions.py
```

**功能:**
- 自动运行两个版本（各30秒）
- 收集 FPS 数据
- 生成对比报告

**输出示例:**
```
📊 性能对比报告
═══════════════════════════════════════
指标          原版      优化版     差异
───────────────────────────────────────
平均FPS      28.5      32.1     +12.6%
最小FPS      25.2      28.4     +12.7%
最大FPS      31.8      35.6     +11.9%
```

**适用场景:**
- 验证优化效果
- 性能测试
- 版本选择

---

## 📊 性能分析工具（3个）

### 8. `profile_performance.py` - 性能剖析

```bash
python3 profile_performance.py
```

分析各组件耗时，找出性能瓶颈。

### 9. `test_onnx_performance.py` - ONNX 基准测试

```bash
python3 test_onnx_performance.py
```

测试 ONNX 推理性能。

### 10. `benchmark_*.py` - 各种基准测试

测试不同配置的性能表现。

---

## 🎯 使用场景快速索引

### 场景1: 每天正常使用

```bash
cd /home/zhuo-skadi/Documents/ros2-robt
./run.sh
```

**就这么简单！**

---

### 场景2: 程序无法启动

```bash
# 1. 诊断问题
./portable_check.sh

# 2. 按提示修复
# ...

# 3. 重新启动
./run.sh
```

---

### 场景3: 摄像头错误码 -6

```bash
# 方案A: 使用修复脚本
./fix_camera_permission.sh

# 方案B: 使用一键启动（自动修复）
./run.sh

# 方案C: 手动修复
pkill -9 -f person_detect
sudo chmod 666 /dev/bus/usb/001/005
```

---

### 场景4: 带到新机器

```bash
# 步骤1: 在原机器保存环境
./save_environment.sh
# 会生成 environment_snapshot_XXX.tar.gz

# 步骤2: 复制文件到新机器
scp environment_snapshot_XXX.tar.gz user@newhost:~/
scp ros2-robt.tar.gz user@newhost:~/

# 步骤3: 在新机器上
tar -xzf ros2-robt.tar.gz
cd ros2-robt
pip install -r requirements.txt

# 步骤4: 诊断和启动
./portable_check.sh
./run.sh
```

---

### 场景5: 比赛现场（临时机器）

```bash
# 快速部署3步走
cd ros2-robt
./portable_check.sh  # 快速诊断
./run.sh             # 自动修复并启动

# 如果没有 sudo 权限，让管理员运行：
sudo chmod 666 /dev/bus/usb/001/005
```

---

### 场景6: 性能优化

```bash
# 1. 安装加速后端
./install_accelerated_backends.sh  # 查看推荐
pip install onnxruntime-openvino   # Intel/AMD 通用

# 2. 性能分析
python3 profile_performance.py

# 3. 版本对比
python3 compare_versions.py

# 4. 验证提升
./run.sh
# 观察 FPS 变化
```

---

### 场景7: 环境对比和调试

```bash
# 在工作机器
./save_environment.sh

# 在问题机器
./portable_check.sh > problem_env.txt

# 对比差异
diff environment_snapshot_XXX/diagnostic_report.txt problem_env.txt
```

---

## 📚 文档速查

| 文档 | 用途 | 适用场景 |
|------|------|----------|
| **QUICK_REFERENCE.txt** | 快速参考卡片 | 打印随身携带 |
| **PORTABLE_SOLUTION.md** | 完整解决方案 | 详细了解原理 |
| **TROUBLESHOOTING.md** | 故障排查手册 | 遇到具体问题 |
| **QUICKSTART_OPTIMIZED.md** | 快速开始指南 | 新手入门 |
| **OPTIMIZATION_COMPLETE_V2.md** | 优化报告 | 了解优化细节 |

---

## ⚡ 快速命令参考

```bash
# 日常使用
./run.sh                                    # 启动程序

# 问题诊断
./portable_check.sh                         # 环境诊断
./fix_camera_permission.sh                  # 修复权限

# 环境管理
./save_environment.sh                       # 保存快照
pip freeze > requirements_exact.txt         # 保存包版本

# 性能优化
pip install onnxruntime-openvino           # 安装加速
python3 profile_performance.py             # 性能分析
python3 compare_versions.py                # 版本对比

# 应急修复
pkill -9 -f person_detect                  # 杀进程
sudo chmod 666 /dev/bus/usb/001/XXX        # 修权限
export LD_LIBRARY_PATH=$(pwd)/libs:$LD_LIBRARY_PATH  # 设环境
```

---

## 🎓 最佳实践

### 1. 养成好习惯

```bash
# ✅ 推荐
cd /path/to/ros2-robt
./run.sh

# ❌ 不推荐
cd ~
python3 /path/to/person_detect.py  # 可能找不到文件
sudo python3 person_detect.py      # 使用系统Python
```

### 2. 定期保存环境快照

```bash
# 每次重大更新后
./save_environment.sh

# 保存到云端或备份盘
cp environment_snapshot_*.tar.gz ~/Backup/
```

### 3. 使用虚拟环境（推荐）

```bash
# 创建虚拟环境
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt

# 每次使用
source venv/bin/activate
./run.sh
```

### 4. 打印快速参考

```bash
# 打印这个文件随身携带
cat QUICK_REFERENCE.txt
```

---

## 🆘 需要帮助？

### 第一步：运行诊断
```bash
./portable_check.sh
```

### 第二步：查看文档
根据问题类型查看对应文档：
- 环境问题 → PORTABLE_SOLUTION.md
- 具体错误 → TROUBLESHOOTING.md
- 性能问题 → OPTIMIZATION_COMPLETE_V2.md

### 第三步：收集信息
```bash
./portable_check.sh > diag.txt
python3 person_detect.py > run.log 2>&1
tar -czf debug-info.tar.gz diag.txt run.log
```

---

## ✨ 记住这个

**最简单的使用方式：**

```bash
cd /home/zhuo-skadi/Documents/ros2-robt
./run.sh
```

**遇到任何问题：**

```bash
./portable_check.sh  # 诊断
./run.sh             # 修复
```

**90%的问题都能通过这两个命令解决！**

---

**制作日期**: 2025-10-14  
**版本**: v1.0  
**维护者**: Zhuo-RM Team
