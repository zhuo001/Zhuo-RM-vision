# 🚨 "带出去就出问题" 故障排查指南

## 常见问题速查表

### 问题1: 摄像头无法打开（错误码-6）

**症状:**
```
[BerxelUvcDriver] [ERROR] uvc_open ret = -6
OpenDevice failed, send public control failed!
```

**原因排查:**
```bash
# 1. 检查是否有其他进程占用
ps aux | grep -E "person_detect|berxel|test_camera"

# 2. 检查USB设备权限
lsusb | grep 0603:0009
ls -l /dev/bus/usb/001/005  # 根据实际Bus/Device号调整
```

**解决方案:**
```bash
# 方案A: 杀死冲突进程
pkill -9 -f person_detect

# 方案B: 修复权限（临时）
sudo chmod 666 /dev/bus/usb/001/005

# 方案C: 重新插拔USB
# 1. 拔出USB线
# 2. 等待3秒
# 3. 重新插入

# 方案D: 使用一键启动脚本（推荐）
./run.sh
```

---

### 问题2: 找不到动态库

**症状:**
```
ImportError: libBerxelHawk.so: cannot open shared object file
```

**原因:** LD_LIBRARY_PATH未设置或不正确

**解决方案:**
```bash
# 临时设置
export LD_LIBRARY_PATH=$(pwd)/libs:$LD_LIBRARY_PATH

# 永久设置（添加到~/.bashrc）
echo 'export LD_LIBRARY_PATH=/path/to/ros2-robt/libs:$LD_LIBRARY_PATH' >> ~/.bashrc
source ~/.bashrc

# 或使用一键启动脚本（自动设置）
./run.sh
```

---

### 问题3: Python模块未找到

**症状:**
```
ModuleNotFoundError: No module named 'onnxruntime'
ModuleNotFoundError: No module named 'cv2'
```

**原因:** 
- Python包未安装
- 虚拟环境未激活
- sudo运行时使用的是系统Python

**解决方案:**
```bash
# 1. 检查当前环境
python3 -c "import sys; print(sys.executable)"
python3 -c "import onnxruntime; print(onnxruntime.__version__)"

# 2. 安装缺失的包
pip install -r requirements.txt

# 3. 如果使用虚拟环境
source yolov8_env/bin/activate  # 先激活
python3 person_detect.py

# 4. 不要用sudo运行（除非必要）
python3 person_detect.py  # 好
sudo python3 person_detect.py  # 坏（使用系统Python）
```

---

### 问题4: 模型文件未找到

**症状:**
```
FileNotFoundError: yolov8n.onnx not found
```

**原因:** 相对路径问题，当前目录不正确

**解决方案:**
```bash
# 1. 检查文件是否存在
ls -lh yolov8n.onnx

# 2. 确保在正确的目录运行
cd /home/zhuo-skadi/Documents/ros2-robt
python3 person_detect.py

# 3. 如果模型缺失，重新导出
python3 -c "from ultralytics import YOLO; YOLO('yolov8n.pt').export(format='onnx', imgsz=416)"
```

---

### 问题5: FPS很低（<15）

**原因排查:**
```bash
# 运行性能分析
python3 profile_performance.py
```

**可能原因:**
1. **只使用CPU推理** → 安装OpenVINO/ROCm
2. **系统负载高** → 关闭其他程序
3. **电源管理限制** → 切换到高性能模式

**解决方案:**
```bash
# 1. 安装加速后端
pip install onnxruntime-openvino  # Intel/AMD通用
# 或
pip install onnxruntime-rocm      # AMD GPU专用

# 2. 检查系统负载
top
htop

# 3. 切换电源模式（笔记本）
# 设置 -> 电源 -> 性能模式 -> 高性能

# 4. 验证加速后端
python3 -c "import onnxruntime as ort; print(ort.get_available_providers())"
# 应该看到: ['OpenVINOExecutionProvider', ...] 或 ['ROCMExecutionProvider', ...]
```

---

### 问题6: 不同机器结果不一致

**原因:** 环境差异

**解决方案:**
```bash
# 1. 导出环境快照（在工作正常的机器上）
pip freeze > requirements_exact.txt
python3 portable_check.sh > environment_snapshot.txt

# 2. 在新机器上对比
python3 portable_check.sh
diff environment_snapshot.txt <(./portable_check.sh)

# 3. 同步环境
pip install -r requirements_exact.txt
```

---

## 完整诊断流程

### 第一步: 运行环境诊断
```bash
chmod +x portable_check.sh
./portable_check.sh
```

如果显示问题，按提示修复后重新运行。

### 第二步: 使用一键启动
```bash
chmod +x run.sh
./run.sh
```

自动修复常见问题并启动程序。

### 第三步: 如果还有问题
```bash
# 完整日志输出
python3 person_detect.py 2>&1 | tee debug.log

# 发送debug.log寻求帮助
```

---

## 不同场景的最佳实践

### 场景1: 在实验室（固定环境）

```bash
# 一次性配置
sudo cp berxel-usb.rules /etc/udev/rules.d/
sudo usermod -aG berxel,video $USER
echo 'export LD_LIBRARY_PATH=/path/to/ros2-robt/libs:$LD_LIBRARY_PATH' >> ~/.bashrc

# 每次运行
cd /path/to/ros2-robt
python3 person_detect.py
```

### 场景2: 带到比赛现场（陌生环境）

```bash
# 推荐：使用一键启动脚本
cd /path/to/ros2-robt
./run.sh

# 备选：手动修复
./portable_check.sh  # 诊断问题
# 按提示修复
python3 person_detect.py
```

### 场景3: 换到不同电脑

```bash
# 1. 打包项目（在源机器）
cd /path/to/ros2-robt
tar -czf ros2-robt-portable.tar.gz \
    *.py *.sh *.onnx \
    libs/ Include/ \
    requirements.txt

# 2. 解压到新电脑
tar -xzf ros2-robt-portable.tar.gz
cd ros2-robt

# 3. 安装依赖
pip install -r requirements.txt

# 4. 运行诊断和启动
./portable_check.sh
./run.sh
```

### 场景4: 树莓派/嵌入式设备

```bash
# 1. 确认Python版本兼容
python3 --version  # 需要 >= 3.8

# 2. 轻量级安装
pip install numpy opencv-python-headless onnxruntime

# 3. 降低分辨率（提升性能）
# 编辑 person_detect.py
# 将 input_size=416 改为 input_size=320

# 4. 使用CPU优化版本
python3 person_detect.py
```

---

## 快速命令参考

### 诊断命令
```bash
./portable_check.sh              # 完整环境诊断
lsusb | grep 0603                # 检查摄像头
python3 -c "import onnxruntime"  # 验证依赖
ps aux | grep person_detect      # 检查进程
```

### 修复命令
```bash
./run.sh                         # 自动修复并启动
./fix_camera_permission.sh       # 修复摄像头权限
pkill -9 -f person_detect        # 杀死冲突进程
export LD_LIBRARY_PATH=$(pwd)/libs:$LD_LIBRARY_PATH  # 设置库路径
```

### 性能优化
```bash
pip install onnxruntime-openvino  # 安装加速后端
python3 profile_performance.py    # 性能分析
python3 compare_versions.py       # 版本对比
```

---

## 应急方案

如果所有方法都失败，使用最小化版本：

```bash
# 创建最小测试脚本
cat > test_minimal.py << 'EOF'
import berxel_camera
cam = berxel_camera.BerxelCamera()
print("相机初始化成功!")
frame = cam.get_frame()
print(f"获取帧成功: {frame.shape if frame is not None else 'None'}")
EOF

python3 test_minimal.py
```

如果这个能运行，说明问题在YOLO或ONNX Runtime，逐步添加功能定位问题。

---

## 预防措施

### 1. 环境快照
```bash
# 在工作正常时保存环境
pip freeze > requirements_working.txt
./portable_check.sh > env_working.txt
```

### 2. 创建虚拟环境（推荐）
```bash
python3 -m venv venv_portable
source venv_portable/bin/activate
pip install -r requirements.txt
```

### 3. Docker方案（终极便携）
```bash
# 构建镜像（一次性）
docker build -t ros2-robt .

# 运行（任何地方）
docker run --rm -it \
    --device=/dev/bus/usb \
    --privileged \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -e DISPLAY=$DISPLAY \
    ros2-robt
```

---

**记住：遇到问题时，先运行 `./portable_check.sh` 诊断，90%的问题都能找到答案！**
