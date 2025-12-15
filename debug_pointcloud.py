#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import struct

class PointCloudDebugger(Node):
    def __init__(self):
        super().__init__('pointcloud_debugger')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/unilidar/cloud',
            self.callback,
            10)
        self.count = 0

    def callback(self, msg):
        self.count += 1
        if self.count % 10 == 0:
            print(f"\n=== PointCloud2 Message #{self.count} ===")
            print(f"Header: {msg.header.frame_id}")
            print(f"Height: {msg.height}, Width: {msg.width}")
            print(f"Point step: {msg.point_step}, Row step: {msg.row_step}")
            print(f"Is dense: {msg.is_dense}, Is bigendian: {msg.is_bigendian}")
            print(f"Fields: {len(msg.fields)}")
            for field in msg.fields:
                print(f"  - {field.name}: offset={field.offset}, datatype={field.datatype}, count={field.count}")
            print(f"Data length: {len(msg.data)} bytes")
            
            # Try to parse first point
            if len(msg.data) >= msg.point_step:
                try:
                    x = struct.unpack('f', bytes(msg.data[0:4]))[0]
                    y = struct.unpack('f', bytes(msg.data[4:8]))[0]
                    z = struct.unpack('f', bytes(msg.data[8:12]))[0]
                    print(f"First point: x={x:.3f}, y={y:.3f}, z={z:.3f}")
                except:
                    pass

def main():
    rclpy.init()
    node = PointCloudDebugger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
