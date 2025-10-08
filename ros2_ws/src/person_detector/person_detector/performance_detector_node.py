#!/usr/bin/env python3
"""
高性能 ROS2 人体检测节点
集成 GPU 加速 + 多线程处理
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import sys
import os
import threading
from queue import Queue
import time

# 导入 Berxel 相机模块
from person_detector.berxel_camera import BerxelCamera

# 导入自定义消息
from person_detector_msgs.msg import PersonDetection, PersonDetectionArray

# 导入 ByteTrack
from person_detector.byte_tracker import ByteTracker


class PerformancePersonDetectorNode(Node):
    """高性能 ROS2 人体检测节点"""
    
    def __init__(self):
        super().__init__('performance_person_detector_node')
        
        # 声明参数
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('conf_threshold', 0.55)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('use_tracking', True)
        self.declare_parameter('use_gpu', True)
        self.declare_parameter('num_threads', 2)
        
        # 获取参数
        model_path = self.get_parameter('model_path').value
        self.conf_threshold = self.get_parameter('conf_threshold').value
        self.iou_threshold = self.get_parameter('iou_threshold').value
        self.publish_debug = self.get_parameter('publish_debug_image').value
        self.use_tracking = self.get_parameter('use_tracking').value
        self.use_gpu = self.get_parameter('use_gpu').value
        num_threads = self.get_parameter('num_threads').value
        
        # 初始化 Berxel 相机
        self.get_logger().info('Initializing Berxel P100R camera...')
        self.camera = BerxelCamera()
        if not self.camera.initialize():
            self.get_logger().error('Failed to initialize Berxel camera!')
            raise RuntimeError('Camera initialization failed')
        self.get_logger().info('Camera initialized successfully')
        
        # 加载 YOLOv8 模型（GPU加速）
        self.get_logger().info(f'Loading YOLO model: {model_path}')
        device = 'cuda:0' if self.use_gpu else 'cpu'
        self.model = YOLO(model_path)
        # 预热模型
        dummy_img = np.zeros((640, 640, 3), dtype=np.uint8)
        _ = self.model(dummy_img, device=device, verbose=False)
        self.get_logger().info(f'YOLO model loaded on {device}')
        
        # 创建发布者
        self.detections_pub = self.create_publisher(
            PersonDetectionArray,
            'person_detections',
            10
        )
        
        if self.publish_debug:
            self.debug_image_pub = self.create_publisher(
                Image,
                'detection_debug_image',
                10
            )
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # 初始化 ByteTracker
        self.tracker = None
        if self.use_tracking:
            self.tracker = ByteTracker(track_thresh=0.5, track_buffer=30, match_thresh=0.8)
            self.get_logger().info('ByteTracker initialized')
        
        # 多线程处理队列
        self.frame_queue = Queue(maxsize=2)  # 相机帧队列
        self.result_queue = Queue(maxsize=2)  # 检测结果队列
        
        # 启动采集线程
        self.running = True
        self.capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.capture_thread.start()
        
        # 启动处理线程
        self.process_threads = []
        for i in range(num_threads):
            t = threading.Thread(target=self._process_loop, args=(device,), daemon=True)
            t.start()
            self.process_threads.append(t)
        
        # 创建定时器用于发布结果
        self.timer = self.create_timer(1.0 / 60.0, self.publish_callback)
        
        # 性能统计
        self.fps_counter = 0
        self.fps_start_time = time.time()
        
        tracking_status = 'enabled' if self.use_tracking else 'disabled'
        self.get_logger().info(
            f'Performance Person Detector Node started '
            f'(GPU: {self.use_gpu}, threads: {num_threads}, tracking: {tracking_status})'
        )
    
    def _capture_loop(self):
        """相机采集线程"""
        while self.running:
            try:
                frame = self.camera.get_frame()
                depth = self.camera.get_depth()
                
                if frame is not None:
                    # 非阻塞放入队列
                    if not self.frame_queue.full():
                        self.frame_queue.put((frame, depth))
                        
            except Exception as e:
                self.get_logger().error(f'Capture error: {e}')
                time.sleep(0.01)
    
    def _process_loop(self, device):
        """检测处理线程"""
        while self.running:
            try:
                # 从队列获取帧
                if self.frame_queue.empty():
                    time.sleep(0.001)
                    continue
                
                frame, depth = self.frame_queue.get(timeout=0.1)
                
                h, w = frame.shape[:2]
                
                # 处理深度图
                depth_resized = None
                if depth is not None and depth.size > 0:
                    depth_resized = cv2.resize(depth, (w, h), interpolation=cv2.INTER_NEAREST)
                
                # 运行 YOLO 检测
                results = self.model(
                    frame,
                    conf=self.conf_threshold,
                    iou=self.iou_threshold,
                    max_det=50,
                    classes=[0],
                    verbose=False,
                    device=device
                )
                
                # 提取有效检测
                raw_detections = []
                for result in results:
                    boxes = result.boxes
                    for box in boxes:
                        try:
                            class_id = int(box.cls[0])
                            if class_id != 0:
                                continue
                            
                            x1, y1, x2, y2 = box.xyxy[0]
                            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                            confidence = float(box.conf)
                            
                            if not self.is_valid_person((x1, y1, x2, y2), confidence, h, w):
                                continue
                            
                            raw_detections.append([x1, y1, x2, y2, confidence])
                            
                        except Exception as e:
                            continue
                
                # 放入结果队列
                if not self.result_queue.full():
                    self.result_queue.put((frame, depth_resized, raw_detections))
                    
            except Exception as e:
                self.get_logger().error(f'Process error: {e}')
                time.sleep(0.01)
    
    def publish_callback(self):
        """发布检测结果"""
        try:
            if self.result_queue.empty():
                return
            
            frame, depth_resized, raw_detections = self.result_queue.get_nowait()
            h, w = frame.shape[:2]
            
            # 创建消息
            detection_array = PersonDetectionArray()
            detection_array.header = Header()
            detection_array.header.stamp = self.get_clock().now().to_msg()
            detection_array.header.frame_id = 'berxel_camera_link'
            
            valid_detections = []
            
            # 如果启用跟踪
            if self.use_tracking and self.tracker is not None and len(raw_detections) > 0:
                tracks = self.tracker.update(raw_detections)
                
                for track in tracks:
                    x1, y1, x2, y2 = map(int, track.xyxy)
                    center_x = int((x1 + x2) / 2)
                    center_y = int((y1 + y2) / 2)
                    
                    depth_value = self.get_depth_at_point(depth_resized, center_x, center_y)
                    
                    detection = PersonDetection()
                    detection.header = detection_array.header
                    detection.track_id = track.track_id
                    detection.x1 = x1
                    detection.y1 = y1
                    detection.x2 = x2
                    detection.y2 = y2
                    detection.center_x = center_x
                    detection.center_y = center_y
                    detection.width = x2 - x1
                    detection.height = y2 - y1
                    detection.confidence = track.score
                    
                    if depth_value is not None:
                        detection.distance = depth_value / 17000.0
                        detection.has_depth = True
                    else:
                        detection.distance = 0.0
                        detection.has_depth = False
                    
                    valid_detections.append(detection)
            
            else:
                # 不使用跟踪
                for det in raw_detections:
                    x1, y1, x2, y2, confidence = det
                    center_x = int((x1 + x2) / 2)
                    center_y = int((y1 + y2) / 2)
                    
                    depth_value = self.get_depth_at_point(depth_resized, center_x, center_y)
                    
                    detection = PersonDetection()
                    detection.header = detection_array.header
                    detection.track_id = -1
                    detection.x1 = int(x1)
                    detection.y1 = int(y1)
                    detection.x2 = int(x2)
                    detection.y2 = int(y2)
                    detection.center_x = center_x
                    detection.center_y = center_y
                    detection.width = int(x2 - x1)
                    detection.height = int(y2 - y1)
                    detection.confidence = confidence
                    
                    if depth_value is not None:
                        detection.distance = depth_value / 17000.0
                        detection.has_depth = True
                    else:
                        detection.distance = 0.0
                        detection.has_depth = False
                    
                    valid_detections.append(detection)
            
            # 设置检测数组
            detection_array.detections = valid_detections
            detection_array.count = len(valid_detections)
            
            # 发布检测结果
            self.detections_pub.publish(detection_array)
            
            # 发布调试图像
            if self.publish_debug and len(valid_detections) > 0:
                self.publish_debug_image(frame, valid_detections)
            
            # FPS统计
            self.fps_counter += 1
            if time.time() - self.fps_start_time >= 1.0:
                fps = self.fps_counter / (time.time() - self.fps_start_time)
                self.get_logger().info(f'FPS: {fps:.1f}')
                self.fps_counter = 0
                self.fps_start_time = time.time()
                
        except Exception as e:
            self.get_logger().error(f'Publish error: {e}')
    
    def is_valid_person(self, box, confidence, frame_height, frame_width):
        """检查检测框是否符合人形特征"""
        x1, y1, x2, y2 = box
        width = x2 - x1
        height = y2 - y1
        
        if width <= 0 or height <= 0:
            return False
        
        aspect_ratio = height / width
        
        if aspect_ratio < 0.5 or aspect_ratio > 5.0:
            return False
        
        area = width * height
        frame_area = frame_height * frame_width
        area_ratio = area / frame_area
        
        if area_ratio < 0.005 or area_ratio > 0.95:
            return False
        
        if width < 60 or height < 100:
            return False
        
        if width > 1850 or height > 1070:
            return False
            
        if confidence < self.conf_threshold:
            return False
        
        return True
    
    def get_depth_at_point(self, depth_map, x, y, window_size=7):
        """获取指定点的深度值"""
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
        
        if depth_value < 3000 or depth_value > 150000:
            return None
        
        return float(depth_value)
    
    def publish_debug_image(self, frame, detections):
        """发布调试图像"""
        try:
            debug_frame = frame.copy()
            
            for det in detections:
                cv2.rectangle(
                    debug_frame,
                    (det.x1, det.y1),
                    (det.x2, det.y2),
                    (0, 255, 0),
                    2
                )
                
                cv2.drawMarker(
                    debug_frame,
                    (det.center_x, det.center_y),
                    (0, 0, 255),
                    cv2.MARKER_CROSS,
                    10,
                    2
                )
                
                if det.track_id >= 0:
                    label = f'ID:{det.track_id} {det.confidence:.2f}'
                else:
                    label = f'Person {det.confidence:.2f}'
                if det.has_depth:
                    label += f' {det.distance:.2f}m'
                
                cv2.putText(
                    debug_frame,
                    label,
                    (det.x1, det.y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    2
                )
            
            image_msg = self.bridge.cv2_to_imgmsg(debug_frame, encoding='bgr8')
            image_msg.header.stamp = self.get_clock().now().to_msg()
            image_msg.header.frame_id = 'berxel_camera_link'
            self.debug_image_pub.publish(image_msg)
            
        except Exception as e:
            self.get_logger().warn(f'Error publishing debug image: {e}')
    
    def destroy_node(self):
        """清理资源"""
        self.get_logger().info('Shutting down performance person detector node...')
        self.running = False
        
        # 等待线程结束
        if hasattr(self, 'capture_thread'):
            self.capture_thread.join(timeout=2.0)
        for t in self.process_threads:
            t.join(timeout=2.0)
        
        if hasattr(self, 'camera'):
            self.camera.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = PerformancePersonDetectorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
