#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
try:
    import open3d as o3d
except ImportError:
    print("Error: open3d library is not installed. Please install it using: pip install open3d")
    exit(1)
import numpy as np
import threading
import time
import signal

class Mid70Visualizer(Node):
    def __init__(self):
        super().__init__('mid70_visualizer')
        # Mid-70 usually publishes to /livox/lidar or /livox/lidar_pointcloud
        self.topic_name = '/livox/lidar' 
        self.subscription = self.create_subscription(
            PointCloud2,
            self.topic_name,
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.pcd = o3d.geometry.PointCloud()
        self.vis = o3d.visualization.Visualizer()
        self.new_data = False
        self.points = None
        self.lock = threading.Lock()
        
        print(f"Waiting for PointCloud2 data on topic: {self.topic_name}")

    def listener_callback(self, msg):
        # Convert ROS PointCloud2 to Open3D PointCloud
        # This generator yields (x, y, z) tuples
        field_names = [field.name for field in msg.fields]
        cloud_data = list(pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z")))
        
        if not cloud_data:
            return

        points = np.array(cloud_data)
        
        with self.lock:
            self.points = points
            self.new_data = True

    def run_visualization(self):
        self.vis.create_window(window_name="Mid-70 Lidar Point Cloud", width=1280, height=720)
        
        # Add a coordinate frame for reference
        axis_pcd = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0, origin=[0, 0, 0])
        self.vis.add_geometry(axis_pcd)
        
        # Initialize point cloud geometry
        self.vis.add_geometry(self.pcd)
        
        # View control
        ctr = self.vis.get_view_control()
        ctr.set_front([-1, 0, 1])
        ctr.set_lookat([0, 0, 0])
        ctr.set_up([0, 0, 1])
        ctr.set_zoom(0.5)

        keep_running = True
        while keep_running:
            with self.lock:
                if self.new_data and self.points is not None:
                    self.pcd.points = o3d.utility.Vector3dVector(self.points)
                    # Optional: Color by height or intensity if available
                    # For now, simple uniform color or height-based
                    # colors = np.zeros_like(self.points)
                    # colors[:, 0] = 1.0 # Red
                    # self.pcd.colors = o3d.utility.Vector3dVector(colors)
                    
                    self.vis.update_geometry(self.pcd)
                    self.new_data = False
            
            keep_running = self.vis.poll_events()
            self.vis.update_renderer()
            time.sleep(0.01)
            
        self.vis.destroy_window()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    
    visualizer_node = Mid70Visualizer()
    
    # Run ROS 2 spinning in a separate thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(visualizer_node,), daemon=True)
    spin_thread.start()
    
    try:
        visualizer_node.run_visualization()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
