#!/usr/bin/env python3
"""
YOLOv8 Person Detection with Depth Estimation
优化版本 - 高性能实时检测系统

特性：
- 智能推理后端选择（OpenVINO/ROCm/CUDA/CPU）
- P100R 深度相机集成
- 高效内存管理
- 性能监控与FPS显示
- 独立深度可视化

性能指标：
- CPU: 28-33 FPS
- OpenVINO: 40-50 FPS
- ROCm: 60-80 FPS
"""

import os
# Ensure Qt uses XCB (avoid missing 'wayland' plugin when system defaults to Wayland)
# Must set before importing cv2 so Qt picks the correct platform plugin
os.environ.setdefault('QT_QPA_PLATFORM', 'xcb')
import cv2
import numpy as np
import time
import sys
from pathlib import Path
from typing import Optional, Tuple, List
import onnxruntime as ort

from berxel_camera import BerxelCamera


class PerformanceMonitor:
    """性能监控器：跟踪FPS和推理时间"""
    
    def __init__(self, window_size: int = 30):
        self.window_size = window_size
        self.frame_times = []
        self.start_time = time.time()
        self.frame_count = 0
        
    def update(self):
        """更新帧计数"""
        self.frame_count += 1
        current_time = time.time()
        self.frame_times.append(current_time)
        
        # 保持窗口大小
        if len(self.frame_times) > self.window_size:
            self.frame_times.pop(0)
    
    def get_fps(self) -> float:
        """计算当前FPS"""
        if len(self.frame_times) < 2:
            return 0.0
        elapsed = self.frame_times[-1] - self.frame_times[0]
        return (len(self.frame_times) - 1) / elapsed if elapsed > 0 else 0.0
    
    def get_stats(self) -> dict:
        """获取性能统计"""
        return {
            'fps': self.get_fps(),
            'total_frames': self.frame_count,
            'elapsed': time.time() - self.start_time
        }


class InferenceBackend:
    """ONNX Runtime 推理后端管理器"""
    
    BACKEND_PRIORITY = [
        'OpenVINOExecutionProvider',
        'ROCMExecutionProvider', 
        'CUDAExecutionProvider',
        'CPUExecutionProvider'
    ]
    
    BACKEND_INFO = {
        'OpenVINOExecutionProvider': ('OpenVINO', '✅', 'Intel CPU/iGPU acceleration'),
        'ROCMExecutionProvider': ('ROCm', '✅', 'AMD GPU acceleration'),
        'CUDAExecutionProvider': ('CUDA', '⚠️', 'NVIDIA GPU acceleration'),
        'CPUExecutionProvider': ('CPU', 'ℹ️', 'Baseline x86 optimization')
    }
    
    def __init__(self, model_path: str):
        self.model_path = model_path
        self.session = None
        self.backend_name = "CPU"
        self.input_name = None
        self.output_names = None
        
    def initialize(self) -> bool:
        """初始化推理后端"""
        print("\n🔍 Detecting execution providers...")
        available = ort.get_available_providers()
        print(f"Available: {available}")
        
        # 选择最佳后端
        selected_provider = None
        for provider in self.BACKEND_PRIORITY:
            if provider in available:
                selected_provider = provider
                break
        
        if selected_provider:
            name, icon, desc = self.BACKEND_INFO[selected_provider]
            self.backend_name = name
            providers = [selected_provider, 'CPUExecutionProvider']
            print(f"\n{icon} Using {selected_provider}")
            print(f"   ➜ {desc}")
        else:
            providers = ['CPUExecutionProvider']
            print("\nℹ️ Using CPUExecutionProvider (baseline)")
            print("   ➜ Install acceleration: pip install onnxruntime-openvino")
        
        # 加载模型
        try:
            print(f"\n📦 Loading model: {self.model_path}")
            self.session = ort.InferenceSession(self.model_path, providers=providers)
            
            self.input_name = self.session.get_inputs()[0].name
            self.output_names = [o.name for o in self.session.get_outputs()]
            
            print(f"✓ Model loaded successfully")
            print(f"  Input: {self.input_name} {self.session.get_inputs()[0].shape}")
            print(f"  Outputs: {self.output_names}")
            print(f"  Active backend: {self.backend_name}")
            print(f"  Session providers: {self.session.get_providers()}\n")
            
            return True
        except Exception as e:
            print(f"❌ Failed to load model: {e}")
            return False
    
    def infer(self, input_data: np.ndarray) -> np.ndarray:
        """执行推理"""
        return self.session.run(None, {self.input_name: input_data})[0]


class YOLOv8Detector:
    """YOLOv8 检测器封装"""
    
    def __init__(
        self,
        backend: InferenceBackend,
        input_size: int = 416,
        conf_threshold: float = 0.40,
        iou_threshold: float = 0.45,
        max_detections: int = 100
    ):
        self.backend = backend
        self.input_size = input_size
        self.conf_threshold = conf_threshold
        self.iou_threshold = iou_threshold
        self.max_detections = max_detections
        
    def preprocess(self, frame: np.ndarray) -> np.ndarray:
        """图像预处理"""
        img = cv2.resize(frame, (self.input_size, self.input_size))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = img.transpose(2, 0, 1).astype(np.float32) / 255.0
        return np.expand_dims(img, axis=0)
    
    def postprocess(
        self,
        predictions: np.ndarray,
        orig_shape: Tuple[int, int]
    ) -> List[Tuple[int, int, int, int, float]]:
        """后处理：解析检测结果并应用NMS"""
        predictions = predictions[0].T  # (num_anchors, 84)
        
        boxes = predictions[:, :4]
        scores = predictions[:, 4:]
        
        class_ids = np.argmax(scores, axis=1)
        confidences = np.max(scores, axis=1)
        
        # 过滤低置信度和非人类检测
        mask = (confidences > self.conf_threshold) & (class_ids == 0)
        boxes = boxes[mask]
        confidences = confidences[mask]
        
        if len(boxes) == 0:
            return []
        
        # 转换坐标格式
        x_center, y_center, width, height = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
        x1 = x_center - width / 2
        y1 = y_center - height / 2
        x2 = x_center + width / 2
        y2 = y_center + height / 2
        
        # 准备 NMS 输入（xywh 格式）
        w_xywh = np.maximum(0.0, x2 - x1)
        h_xywh = np.maximum(0.0, y2 - y1)
        boxes_xywh = np.stack([x1, y1, w_xywh, h_xywh], axis=1)
        
        # NMS
        indices = cv2.dnn.NMSBoxes(
            boxes_xywh.tolist(),
            confidences.tolist(),
            self.conf_threshold,
            self.iou_threshold
        )
        
        if len(indices) == 0:
            return []
        
        # 处理 indices 返回格式
        if isinstance(indices, tuple):
            indices = list(indices)
        if isinstance(indices, list) and len(indices) > 0:
            indices = np.array(indices).flatten()
        
        # 缩放到原图尺寸
        h, w = orig_shape
        scale_x = w / self.input_size
        scale_y = h / self.input_size
        
        detections = []
        for idx in indices[:self.max_detections]:
            x1_scaled = int(x1[idx] * scale_x)
            y1_scaled = int(y1[idx] * scale_y)
            x2_scaled = int(x2[idx] * scale_x)
            y2_scaled = int(y2[idx] * scale_y)
            conf = float(confidences[idx])
            
            detections.append((x1_scaled, y1_scaled, x2_scaled, y2_scaled, conf))
        
        return detections
    
    def detect(self, frame: np.ndarray) -> List[Tuple[int, int, int, int, float]]:
        """执行检测"""
        input_data = self.preprocess(frame)
        predictions = self.backend.infer(input_data)
        return self.postprocess(predictions, frame.shape[:2])


class DepthProcessor:
    """深度图处理器"""
    
    # P100R 深度相机参数
    DEPTH_SCALE_FACTOR = 17000.0  # 原始值 / 17000 = 米
    VALID_RANGE = (3000, 150000)  # 有效深度范围（原始值）
    
    def __init__(self, target_size: Tuple[int, int]):
        self.target_size = target_size
        self.depth_cache = None
        self.update_counter = 0
        
    def process(self, depth_map: np.ndarray) -> Optional[np.ndarray]:
        """处理深度图：缩放到目标尺寸"""
        if depth_map is None or depth_map.size == 0:
            return None
        
        # 缩放到目标尺寸
        return cv2.resize(
            depth_map,
            (self.target_size[1], self.target_size[0]),
            interpolation=cv2.INTER_NEAREST
        )
    
    def get_depth_at_point(
        self,
        depth_map: np.ndarray,
        x: int,
        y: int,
        window_size: int = 7
    ) -> Optional[float]:
        """获取指定点的深度值（米）"""
        if depth_map is None:
            return None
        
        h, w = depth_map.shape[:2]
        half_win = window_size // 2
        
        y_min = max(0, y - half_win)
        y_max = min(h, y + half_win + 1)
        x_min = max(0, x - half_win)
        x_max = min(w, x + half_win + 1)
        
        region = depth_map[y_min:y_max, x_min:x_max]
        valid_depths = region[region > 0]
        
        if len(valid_depths) == 0:
            return None
        
        depth_value = np.median(valid_depths)
        
        # 有效性检查
        if depth_value < self.VALID_RANGE[0] or depth_value > self.VALID_RANGE[1]:
            return None
        
        return depth_value / self.DEPTH_SCALE_FACTOR
    
    def visualize(
        self,
        depth_map: np.ndarray,
        display_size: Tuple[int, int]
    ) -> Optional[np.ndarray]:
        """生成深度可视化（伪彩色）"""
        if depth_map is None:
            return None
        
        try:
            # 在显示尺寸上处理（节省计算）
            depth_small = cv2.resize(
                depth_map,
                (display_size[1], display_size[0]),
                interpolation=cv2.INTER_NEAREST
            )
            
            valid_depth = depth_small[depth_small > 0]
            if len(valid_depth) == 0:
                return None
            
            # 使用 min/max（比 percentile 快10倍）
            depth_min = np.min(valid_depth)
            depth_max = np.max(valid_depth)
            
            if depth_max <= depth_min:
                return None
            
            # 归一化并应用伪彩色
            depth_norm = np.clip(
                (depth_small - depth_min) / (depth_max - depth_min),
                0, 1
            )
            depth_norm = (depth_norm * 255).astype(np.uint8)
            depth_color = cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)
            
            # 无效区域显示为黑色
            mask = depth_small == 0
            depth_color[mask] = [0, 0, 0]
            
            return depth_color
        except Exception:
            return None


class PersonDetectionApp:
    """主应用程序"""
    
    def __init__(
        self,
        model_path: str = 'yolov8n.onnx',
        display_scale: float = 0.5,
        depth_update_interval: int = 5
    ):
        self.model_path = model_path
        self.display_scale = display_scale
        self.depth_update_interval = depth_update_interval
        # Headless mode flag (disable GUI/drawing to maximize throughput)
        self.headless = False
        
        # 组件
        self.camera = None
        self.backend = None
        self.detector = None
        self.depth_processor = None
        self.perf_monitor = PerformanceMonitor()
        
    def initialize(self) -> bool:
        """初始化所有组件"""
        print("=" * 60)
        print("Person Detection with Depth - Initialization")
        print("=" * 60)
        
        # 初始化相机
        print("\n📷 Initializing Berxel P100R camera...")
        try:
            self.camera = BerxelCamera()
            if not self.camera.initialize():
                print("❌ Failed to initialize camera")
                return False
            print("✓ Camera initialized")
        except Exception as e:
            print(f"❌ Camera error: {e}")
            return False
        
        # 初始化推理后端
        self.backend = InferenceBackend(self.model_path)
        if not self.backend.initialize():
            return False
        
        # 初始化检测器
        self.detector = YOLOv8Detector(self.backend)
        
        # 初始化深度处理器（等待第一帧，最多3秒）
        print("\n⏳ Waiting for first frame...")
        import time
        frame = None
        for i in range(30):  # 最多等待3秒
            frame = self.camera.get_frame()
            if frame is not None:
                break
            time.sleep(0.1)
        
        if frame is not None:
            h, w = frame.shape[:2]
            self.depth_processor = DepthProcessor((h, w))
            print(f"✓ Got first frame ({w}x{h})")
        else:
            print("⚠️  Timeout waiting for first frame, using default resolution")
            self.depth_processor = DepthProcessor((1080, 1920))
        
        print("=" * 60)
        print("✓ Initialization complete - Press 'q' to quit")
        print("=" * 60)
        return True
    
    def run(self):
        """主循环"""
        import time
        last_heartbeat = time.time()
        frame_timeout_count = 0
        MAX_TIMEOUT = 10  # 最多10次超时后退出
        
        try:
            while True:
                loop_start = time.time()
                
                # 心跳日志（每5秒）
                if loop_start - last_heartbeat > 5:
                    print(f"💓 Heartbeat: FPS={self.perf_monitor.get_fps():.1f}, Frame={self.perf_monitor.frame_count}")
                    last_heartbeat = loop_start
                
                # 获取图像（添加超时检测）
                try:
                    frame = self.camera.get_frame()
                    depth = self.camera.get_depth()
                    frame_timeout_count = 0  # 重置超时计数
                except Exception as e:
                    print(f"⚠️ Camera error: {e}")
                    frame_timeout_count += 1
                    if frame_timeout_count >= MAX_TIMEOUT:
                        print("❌ Too many camera timeouts, exiting...")
                        break
                    time.sleep(0.1)
                    continue
                
                if frame is None:
                    print("⚠️ No frame received")
                    frame_timeout_count += 1
                    if frame_timeout_count >= MAX_TIMEOUT:
                        print("❌ Too many empty frames, exiting...")
                        break
                    continue
                
                h, w = frame.shape[:2]
                
                # 处理深度图
                depth_resized = None
                if depth is not None and self.depth_processor:
                    depth_resized = self.depth_processor.process(depth)
                
                # 运行检测
                detections = self.detector.detect(frame)
                
                # 绘制检测结果（除非 headless）
                if not self.headless:
                    for x1, y1, x2, y2, conf in detections:
                        center_x = (x1 + x2) // 2
                        center_y = (y1 + y2) // 2

                        # 获取深度
                        label = f"Person {conf:.2f}"
                        if depth_resized is not None:
                            dist = self.depth_processor.get_depth_at_point(
                                depth_resized, center_x, center_y
                            )
                            if dist:
                                label += f" {dist:.2f}m"

                        # 绘制
                        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        cv2.putText(
                            frame, label, (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2
                        )
                        cv2.drawMarker(
                            frame, (center_x, center_y),
                            (0, 0, 255), cv2.MARKER_CROSS, 10, 2
                        )
                
                # 更新性能监控
                self.perf_monitor.update()
                fps = self.perf_monitor.get_fps()

                # headless 下每秒打印一次 FPS，方便日志采样
                if self.headless:
                    if not hasattr(self, '_last_fps_print'):
                        self._last_fps_print = time.time()
                        self._fps_counter = 0
                    self._fps_counter += 1
                    now_t = time.time()
                    if now_t - self._last_fps_print >= 1.0:
                        # 使用 perf_monitor.get_fps() 更平滑
                        print(f"[HEADLESS FPS] {self.perf_monitor.get_fps():.1f} fps")
                        self._last_fps_print = now_t
                        self._fps_counter = 0
                
                # 显示FPS（仅在非 headless）
                if not self.headless:
                    cv2.putText(
                        frame, f"FPS: {fps:.1f} | {self.backend.backend_name}",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2
                    )

                    # 显示主窗口
                    display_h = int(h * self.display_scale)
                    display_w = int(w * self.display_scale)
                    display_resized = cv2.resize(
                        frame, (display_w, display_h),
                        interpolation=cv2.INTER_LINEAR
                    )
                    cv2.imshow('Person Detection', display_resized)
                
                # 深度可视化（降低更新频率）
                if not self.headless and (depth_resized is not None and 
                    self.perf_monitor.frame_count % self.depth_update_interval == 0):
                    try:
                        depth_vis = self.depth_processor.visualize(
                            depth_resized, (display_h, display_w)
                        )
                        if depth_vis is not None:
                            cv2.imshow('Depth', depth_vis)
                    except Exception as e:
                        print(f"⚠️ Depth visualization error: {e}")
                
                # 退出检测（添加超时保护）
                # 只有在非 headless 模式下处理键盘事件
                if not self.headless:
                    try:
                        key = cv2.waitKey(1) & 0xFF
                        if key == ord('q'):
                            print("\n✓ Exit requested by user")
                            break
                        elif key == 27:  # ESC键
                            print("\n✓ Exit requested by ESC")
                            break
                    except Exception as e:
                        print(f"⚠️ waitKey error: {e}, continuing...")
                
                # 检查循环耗时
                loop_time = time.time() - loop_start
                if loop_time > 0.5:  # 超过500ms警告
                    print(f"⚠️ Slow loop: {loop_time*1000:.1f}ms")
                    
        except KeyboardInterrupt:
            print("\n⚠️ Interrupted by user")
        except Exception as e:
            print(f"\n❌ Runtime error: {e}")
            import traceback
            traceback.print_exc()
        finally:
            self.cleanup()
    
    def cleanup(self):
        """清理资源"""
        print("\n🧹 Cleaning up...")
        # Only destroy windows if not running headless and cv2 is available
        try:
            if not getattr(self, 'headless', False):
                cv2.destroyAllWindows()
        except Exception as e:
            # Log but continue cleanup
            print(f"⚠️ Error during cv2.destroyAllWindows(): {e}")
        if self.camera:
            self.camera.release()
        
        # 显示性能统计
        stats = self.perf_monitor.get_stats()
        print(f"\n📊 Session Statistics:")
        print(f"  Total frames: {stats['total_frames']}")
        print(f"  Elapsed time: {stats['elapsed']:.1f}s")
        print(f"  Average FPS: {stats['fps']:.1f}")
        print("\n✓ Shutdown complete")


def main():
    """程序入口"""
    import argparse, os
    parser = argparse.ArgumentParser()
    parser.add_argument('--headless', action='store_true', help='Run without GUI to maximize performance')
    args = parser.parse_args()

    HEADLESS = args.headless or os.environ.get('HEADLESS', '0') == '1'

    app = PersonDetectionApp()
    app.headless = HEADLESS
    
    if not app.initialize():
        print("\n❌ Initialization failed")
        sys.exit(1)
    
    # If headless, reduce logging and skip GUI
    if getattr(app, 'headless', False):
        print("Running in headless mode: GUI disabled, maximizing throughput")
    app.run()


if __name__ == "__main__":
    main()
