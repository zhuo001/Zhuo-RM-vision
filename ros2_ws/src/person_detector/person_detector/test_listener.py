#!/usr/bin/env python3
"""
ROS2 人体检测话题监听测试
"""

import rclpy
from rclpy.node import Node
from person_detector_msgs.msg import PersonDetectionArray


class DetectionListenerNode(Node):
    """检测结果监听节点"""
    
    def __init__(self):
        super().__init__('detection_listener')
        
        self.subscription = self.create_subscription(
            PersonDetectionArray,
            'person_detections',
            self.detection_callback,
            10
        )
        
        self.get_logger().info('Detection Listener Node started')
        self.get_logger().info('Listening to /person_detections topic...')
    
    def detection_callback(self, msg):
        """处理检测结果"""
        if msg.count == 0:
            return
        
        self.get_logger().info(f'\n{"="*60}')
        self.get_logger().info(f'Detected {msg.count} person(s):')
        
        for i, det in enumerate(msg.detections):
            info = f'  [{i+1}] '
            
            # 跟踪ID
            if det.track_id >= 0:
                info += f'ID:{det.track_id} '
            
            # 位置和尺寸
            info += f'bbox:[{det.x1},{det.y1},{det.x2},{det.y2}] '
            info += f'size:{det.width}x{det.height} '
            
            # 置信度
            info += f'conf:{det.confidence:.2f} '
            
            # 距离
            if det.has_depth:
                info += f'dist:{det.distance:.2f}m'
            else:
                info += 'dist:N/A'
            
            self.get_logger().info(info)
        
        self.get_logger().info(f'{"="*60}\n')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = DetectionListenerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
