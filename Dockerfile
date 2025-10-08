```dockerfile
# Dockerfile for Berxel P100R Person Detection
# 适配AMD 780M核显，使用ONNXRuntime + OpenVINO优化
# 基于Ubuntu 22.04

FROM ubuntu:22.04

# 设置环境变量
ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONUNBUFFERED=1
ENV TZ=Asia/Shanghai

# 创建工作目录
WORKDIR /workspace

# 安装系统依赖
RUN apt-get update && apt-get install -y \
    # 基础工具
    wget \
    curl \
    git \
    build-essential \
    cmake \
    pkg-config \
    # Python
    python3.10 \
    python3-pip \
    python3-dev \
    # OpenCV依赖
    libopencv-dev \
    python3-opencv \
    libgtk-3-dev \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    libv4l-dev \
    libxvidcore-dev \
    libx264-dev \
    # USB相机支持
    libusb-1.0-0-dev \
    v4l-utils \
    # X11显示支持（用于cv2.imshow）
    libx11-dev \
    libxext-dev \
    x11-apps \
    # OpenCL / ICD (用于 GPU / iGPU 支持，针对 AMD iGPU)
    ocl-icd-libopencl1 \
    ocl-icd-opencl-dev \
    mesa-opencl-icd \
    opencl-headers \
    && rm -rf /var/lib/apt/lists/*

# 升级pip
RUN python3 -m pip install --upgrade pip setuptools wheel

# 安装Python依赖
COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt

# 安装 OpenVINO 运行时与 ONNXRuntime-OpenVINO（优先尝试安装，失败则回退到 CPU）
# 说明：针对 AMD iGPU，本镜像尝试安装常见的 OpenCL/ICD 软件包并通过 pip 安装 OpenVINO Python 运行时。
RUN set -eux; \
    pip3 install --no-cache-dir openvino openvino-dev || echo "警告: openvino pip 包安装失败"; \
    # 安装与 openvino 匹配的 onnxruntime / onnxruntime-openvino（如 wheel 可用）
    pip3 install --no-cache-dir onnxruntime==1.22.0 onnxruntime-openvino==1.22.0 || \
    (echo "警告: onnxruntime-openvino 安装失败，容器将回退到 CPUExecutionProvider" && true)

# 复制项目文件
COPY . /workspace/

# 设置相机设备权限
# 运行容器时需要: --device=/dev/bus/usb
RUN echo '# Berxel Camera udev rules' > /etc/udev/rules.d/99-berxel.rules && \
    echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="2bc5", MODE="0666"' >> /etc/udev/rules.d/99-berxel.rules

# 暴露可能需要的端口（如果添加Web界面）
# EXPOSE 8080

# 设置入口点
ENTRYPOINT ["python3"]
CMD ["person_detect.py", "--help"]

# 构建命令:
#   docker build -t person-detector:amd -f Dockerfile .
#
# 运行命令（需要X11显示和USB设备）:
#   # 允许Docker访问X11显示
#   xhost +local:docker
#
#   # 运行容器
#   docker run -it --rm \
#     --device=/dev/bus/usb \
#     --env="DISPLAY=$DISPLAY" \
#     --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
#     --volume="$PWD:/workspace" \
#     person-detector:amd \
#     person_detect.py --engine onnx-openvino --benchmark
#
# 说明:
#   --device=/dev/bus/usb          - 映射USB设备（Berxel相机）
#   --env="DISPLAY=$DISPLAY"        - 传递显示环境变量
#   --volume="/tmp/.X11-unix:..."   - 映射X11 socket（用于显示窗口）
#   --volume="$PWD:/workspace"      - 映射当前目录（可选，用于开发）

```