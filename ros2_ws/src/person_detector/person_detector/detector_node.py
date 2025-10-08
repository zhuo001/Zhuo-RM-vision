#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import torch

class PersonDetectorNode(Node):
    def __init__(self):
        super().__init__('person_detector')
        
        # 创建订阅者
        self.color_sub = self.create_subscription(
            Image,
            '/berxel_camera/color/image_raw',
            self.color_callback,
            10)
        self.depth_sub = self.create_subscription(
            Image,
            '/berxel_camera/depth/image_raw',
            self.depth_callback,
            10)
            
        # 创建发布者
        self.detections_pub = self.create_publisher(
            Detection2DArray,
            '~/detections',
            10)
        self.debug_image_pub = self.create_publisher(
            Image,
            '~/debug_image',
            10)
            
        # 初始化YOLOv8模型
        self.model = YOLO('yolov8n.pt')
        self.model.classes = [0]  # 只检测人类类别
        
        # 初始化CV bridge
        self.bridge = CvBridge()
        
        # 存储最新的深度图像
        self.latest_depth_image = None
        
        self.get_logger().info('Person detector node initialized')
        
    def color_callback(self, msg):
        try:
            # 将ROS图像消息转换为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 运行目标检测
            results = self.model(cv_image)
            
            # 创建检测消息
            detections_msg = Detection2DArray()
            detections_msg.header = msg.header
            
            # 在图像上处理检测结果
            for result in results:
                boxes = result.boxes
                for box in boxes:
                    # 获取边界框坐标
                    x1, y1, x2, y2 = box.xyxy[0]
                    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(x2)
                    
                    # 计算中心点
                    center_x = int((x1 + x2) / 2)
                    center_y = int((y1 + y2) / 2)
                    
                    # 获取置信度
                    confidence = float(box.conf)
                    
                    # 获取深度信息（如果可用）
                    depth_value = None
                    if self.latest_depth_image is not None:
                        depth_value = self.latest_depth_image[center_y, center_x]
                    
                    # 创建检测消息
                    detection = Detection2D()
                    detection.bbox.center.x = float(center_x)
                    detection.bbox.center.y = float(center_y)
                    detection.bbox.size_x = float(x2 - x1)
                    detection.bbox.size_y = float(y2 - y1)
                    
                    hypothesis = ObjectHypothesisWithPose()
                    hypothesis.hypothesis.class_id = "person"
                    hypothesis.hypothesis.score = confidence
                    detection.results.append(hypothesis)
                    
                    detections_msg.detections.append(detection)
                    
                    # 在图像上绘制检测结果
                    cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    label = f"Person {confidence:.2f}"
                    if depth_value is not None:
                        label += f" {depth_value}mm"
                    cv2.putText(cv_image, label, (x1, y1 - 10),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # 发布检测结果
            self.detections_pub.publish(detections_msg)
            
            # 发布调试图像
            debug_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            debug_msg.header = msg.header
            self.debug_image_pub.publish(debug_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')
    
    def depth_callback(self, msg):
        try:
            # 将ROS深度图像消息转换为OpenCV格式
            self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg)
        except Exception as e:
            self.get_logger().error(f'Error processing depth image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = PersonDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()