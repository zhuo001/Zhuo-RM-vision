#!/usr/bin/env python3
"""
ROS2 人体检测节点
集成 Berxel P100R 深度相机 + YOLOv8 + ByteTrack 跟踪
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

# 导入 Berxel 相机模块
from person_detector.berxel_camera import BerxelCamera

# 导入自定义消息
from person_detector_msgs.msg import PersonDetection, PersonDetectionArray

# 导入 ByteTrack
from person_detector.byte_tracker import ByteTracker


class PersonDetectorNode(Node):
    """ROS2 人体检测节点"""
    
    def __init__(self):
        super().__init__('person_detector_node')
        
        # 声明参数
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('conf_threshold', 0.55)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('use_tracking', False)  # 稍后启用
        
        # 获取参数
        model_path = self.get_parameter('model_path').value
        self.conf_threshold = self.get_parameter('conf_threshold').value
        self.iou_threshold = self.get_parameter('iou_threshold').value
        self.publish_debug = self.get_parameter('publish_debug_image').value
        self.use_tracking = self.get_parameter('use_tracking').value
        
        # 初始化 Berxel 相机
        self.get_logger().info('Initializing Berxel P100R camera...')
        self.camera = BerxelCamera()
        if not self.camera.initialize():
            self.get_logger().error('Failed to initialize Berxel camera!')
            raise RuntimeError('Camera initialization failed')
        self.get_logger().info('Camera initialized successfully')
        
        # 加载 YOLOv8 模型
        self.get_logger().info(f'Loading YOLO model: {model_path}')
        self.model = YOLO(model_path)
        self.get_logger().info('YOLO model loaded')
        
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
        
        # 初始化 ByteTracker（如果启用）
        self.tracker = None
        if self.use_tracking:
            self.tracker = ByteTracker(track_thresh=0.5, track_buffer=30, match_thresh=0.8)
            self.get_logger().info('ByteTracker initialized')
        
        # 创建定时器（30Hz）
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)
        
        tracking_status = 'enabled' if self.use_tracking else 'disabled'
        self.get_logger().info(f'Person Detector Node started successfully (tracking: {tracking_status})')
        
    def is_valid_person(self, box, confidence, frame_height, frame_width):
        """
        检查检测框是否符合人形特征
        """
        x1, y1, x2, y2 = box
        width = x2 - x1
        height = y2 - y1
        
        if width <= 0 or height <= 0:
            return False
        
        # 计算宽高比
        aspect_ratio = height / width
        
        # 人的宽高比限制
        MIN_ASPECT_RATIO = 0.5
        MAX_ASPECT_RATIO = 5.0
        
        if aspect_ratio < MIN_ASPECT_RATIO or aspect_ratio > MAX_ASPECT_RATIO:
            return False
        
        # 面积限制
        area = width * height
        frame_area = frame_height * frame_width
        area_ratio = area / frame_area
        
        if area_ratio < 0.005 or area_ratio > 0.95:
            return False
        
        # 绝对尺寸限制
        if width < 60 or height < 100:
            return False
        
        if width > 1850 or height > 1070:
            return False
            
        # 置信度阈值
        if confidence < self.conf_threshold:
            return False
        
        return True
    
    def get_depth_at_point(self, depth_map, x, y, window_size=7):
        """
        获取指定点的深度值，使用中位数滤波
        """
        if depth_map is None:
            return None
        
        h, w = depth_map.shape[:2]
        
        # 计算采样区域
        half_win = window_size // 2
        y_min = max(0, y - half_win)
        y_max = min(h, y + half_win + 1)
        x_min = max(0, x - half_win)
        x_max = min(w, x + half_win + 1)
        
        # 提取区域
        region = depth_map[y_min:y_max, x_min:x_max]
        
        # 过滤掉无效值
        valid_depths = region[region > 0]
        
        if len(valid_depths) == 0:
            return None
        
        # 使用中位数
        depth_value = np.median(valid_depths)
        
        # P100R 深度范围检查：3000-150000 原始值（0.3m-8m）
        if depth_value < 3000 or depth_value > 150000:
            return None
        
        return float(depth_value)
    
    def timer_callback(self):
        """定时器回调：获取图像并处理"""
        try:
            # 获取相机数据
            frame = self.camera.get_frame()
            depth = self.camera.get_depth()
            
            if frame is None:
                self.get_logger().warn('No frame received from camera')
                return
            
            h, w = frame.shape[:2]
            
            # 处理深度图
            depth_resized = None
            if depth is not None and depth.size > 0:
                # 将深度图缩放到彩色图尺寸
                depth_resized = cv2.resize(depth, (w, h), interpolation=cv2.INTER_NEAREST)
            
            # 运行 YOLO 检测
            results = self.model(
                frame,
                conf=self.conf_threshold,
                iou=self.iou_threshold,
                max_det=50,
                classes=[0],  # 只检测人类
                verbose=False
            )
            
            # 创建消息
            detection_array = PersonDetectionArray()
            detection_array.header = Header()
            detection_array.header.stamp = self.get_clock().now().to_msg()
            detection_array.header.frame_id = 'berxel_camera_link'
            
            valid_detections = []
            raw_detections = []  # 用于跟踪的原始检测
            
            # 处理每个检测结果
            for result in results:
                boxes = result.boxes
                for box in boxes:
                    try:
                        # 检查类别ID
                        class_id = int(box.cls[0])
                        if class_id != 0:
                            continue
                        
                        # 获取边界框
                        x1, y1, x2, y2 = box.xyxy[0]
                        x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                        
                        # 获取置信度
                        confidence = float(box.conf)
                        
                        # 验证人形特征
                        if not self.is_valid_person((x1, y1, x2, y2), confidence, h, w):
                            continue
                        
                        # 保存原始检测用于跟踪 [x1, y1, x2, y2, score]
                        raw_detections.append([x1, y1, x2, y2, confidence])
                        
                    except Exception as e:
                        self.get_logger().warn(f'Error processing detection: {e}')
                        continue
            
            # 如果启用跟踪，使用 ByteTracker
            if self.use_tracking and self.tracker is not None and len(raw_detections) > 0:
                tracks = self.tracker.update(raw_detections)
                
                # 为每个跟踪创建消息
                for track in tracks:
                    x1, y1, x2, y2 = map(int, track.xyxy)
                    center_x = int((x1 + x2) / 2)
                    center_y = int((y1 + y2) / 2)
                    
                    # 获取深度
                    depth_value = self.get_depth_at_point(depth_resized, center_x, center_y)
                    
                    # 创建检测消息
                    detection = PersonDetection()
                    detection.header = detection_array.header
                    detection.track_id = track.track_id  # 使用跟踪ID
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
                        # P100R 深度转换：原始值 / 17000 = 米
                        detection.distance = depth_value / 17000.0
                        detection.has_depth = True
                    else:
                        detection.distance = 0.0
                        detection.has_depth = False
                    
                    valid_detections.append(detection)
            
            else:
                # 不使用跟踪，直接输出检测结果
                for det in raw_detections:
                    x1, y1, x2, y2, confidence = det
                    center_x = int((x1 + x2) / 2)
                    center_y = int((y1 + y2) / 2)
                    
                    # 获取深度
                    depth_value = self.get_depth_at_point(depth_resized, center_x, center_y)
                    
                    # 创建检测消息
                    detection = PersonDetection()
                    detection.header = detection_array.header
                    detection.track_id = -1  # 未启用跟踪
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
                        # P100R 深度转换：原始值 / 17000 = 米
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
            
        except Exception as e:
            self.get_logger().error(f'Error in timer callback: {e}')
    
    def publish_debug_image(self, frame, detections):
        """发布带检测框的调试图像"""
        try:
            debug_frame = frame.copy()
            
            for det in detections:
                # 绘制边界框
                cv2.rectangle(
                    debug_frame,
                    (det.x1, det.y1),
                    (det.x2, det.y2),
                    (0, 255, 0),
                    2
                )
                
                # 绘制中心点
                cv2.drawMarker(
                    debug_frame,
                    (det.center_x, det.center_y),
                    (0, 0, 255),
                    cv2.MARKER_CROSS,
                    10,
                    2
                )
                
                # 添加标签
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
            
            # 转换并发布
            image_msg = self.bridge.cv2_to_imgmsg(debug_frame, encoding='bgr8')
            image_msg.header.stamp = self.get_clock().now().to_msg()
            image_msg.header.frame_id = 'berxel_camera_link'
            self.debug_image_pub.publish(image_msg)
            
        except Exception as e:
            self.get_logger().warn(f'Error publishing debug image: {e}')
    
    def destroy_node(self):
        """清理资源"""
        self.get_logger().info('Shutting down person detector node...')
        if hasattr(self, 'camera'):
            self.camera.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = PersonDetectorNode()
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
