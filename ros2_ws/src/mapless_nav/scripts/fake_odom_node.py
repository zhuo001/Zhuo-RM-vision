#!/usr/bin/env python3
"""
Fake Odometry Publisher for testing Nav2 without real odometry hardware.
Publishes static odom -> base_link transform and /odom topic.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math


class FakeOdomNode(Node):
    def __init__(self):
        super().__init__('fake_odom_node')
        
        # Parameters
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        
        self.publish_rate = self.get_parameter('publish_rate').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timer
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_odom)
        
        self.get_logger().info(f'Fake odom node started: {self.odom_frame} -> {self.base_frame}')
    
    def publish_odom(self):
        now = self.get_clock().now().to_msg()
        
        # Publish TF: odom -> base_link
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)
        
        # Publish Odometry message
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = 0.0
        odom.pose.pose.position.y = 0.0
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.w = 1.0
        odom.twist.twist.linear.x = 0.0
        odom.twist.twist.angular.z = 0.0
        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = FakeOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
