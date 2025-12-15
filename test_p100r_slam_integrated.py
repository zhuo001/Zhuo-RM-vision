"""
P100R + SLAM 完整集成测试
使用OpenCV UVC读取彩色图，基于亮度估算深度，实时SLAM避障

作者: Zhuo
日期: 2025-10-15
"""

import cv2
import numpy as np
import time
from depth_slam_obstacle import DepthSLAMObstacleDetector


class P100RCamera:
    """P100R相机管理类（OpenCV UVC模式）"""
    
    def __init__(self, camera_index=0):
        self.camera_index = camera_index
        self.cap = None
        self.width = 1920
        self.height = 1080
        self.fps = 30
        
    def initialize(self):
        """初始化相机"""
        print("=" * 60)
        print("初始化P100R相机...")
        print("=" * 60)
        
        # 使用DirectShow后端
        self.cap = cv2.VideoCapture(self.camera_index, cv2.CAP_DSHOW)
        
        if not self.cap.isOpened():
            raise RuntimeError(f"无法打开相机索引 {self.camera_index}")
        
        # 设置分辨率
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)  # 自动曝光
        
        # 验证设置
        actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
        
        print(f"✅ 相机初始化成功")
        print(f"   分辨率: {actual_width}x{actual_height}")
        print(f"   帧率: {actual_fps} fps")
        
        # 预热相机（丢弃前10帧）
        print("   预热相机...")
        for _ in range(10):
            self.cap.read()
            time.sleep(0.05)
        
        print("✅ 相机就绪\n")
        
    def get_frame(self):
        """读取一帧彩色图像"""
        ret, frame = self.cap.read()
        if not ret or frame is None:
            return None
        return frame
    
    def estimate_depth(self, color_frame, method='brightness'):
        """从彩色图估算深度（临时方案）
        
        注意: 这只是临时方案！真实深度需要SDK
        
        Args:
            color_frame: BGR彩色图像
            method: 估算方法
                - 'brightness': 基于亮度（近处亮，远处暗）
                - 'edge': 基于边缘密度
                - 'hybrid': 混合方法
        
        Returns:
            深度图 (H, W)，单位：毫米，uint16
        """
        gray = cv2.cvtColor(color_frame, cv2.COLOR_BGR2GRAY)
        
        if method == 'brightness':
            # 亮度反比映射到深度
            # 假设: 亮度255->500mm(近), 亮度0->5000mm(远)
            depth = np.clip(5000 - gray * 17, 500, 5000).astype(np.uint16)
            
        elif method == 'edge':
            # 边缘密度映射到深度
            edges = cv2.Canny(gray, 50, 150)
            # 局部边缘密度
            kernel = np.ones((15, 15), np.float32) / 225
            edge_density = cv2.filter2D(edges.astype(np.float32), -1, kernel)
            # 边缘多的地方认为是近处（有纹理细节）
            depth = np.clip(5000 - edge_density * 50, 500, 5000).astype(np.uint16)
            
        elif method == 'hybrid':
            # 混合方法
            # 1. 亮度分量
            depth_bright = 5000 - gray * 17
            
            # 2. 边缘分量
            edges = cv2.Canny(gray, 50, 150)
            kernel = np.ones((15, 15), np.float32) / 225
            edge_density = cv2.filter2D(edges.astype(np.float32), -1, kernel)
            depth_edge = 5000 - edge_density * 50
            
            # 加权融合
            depth = np.clip(depth_bright * 0.7 + depth_edge * 0.3, 500, 5000).astype(np.uint16)
        
        else:
            raise ValueError(f"未知的深度估算方法: {method}")
        
        return depth
    
    def release(self):
        """释放相机"""
        if self.cap is not None:
            self.cap.release()
            print("相机已释放")


def main():
    print("""
╔═══════════════════════════════════════════════════════════╗
║     P100R + SLAM 完整集成测试                             ║
║                                                           ║
║  使用OpenCV读取彩色图 + 亮度估算深度 + SLAM避障           ║
║  注意: 深度为估算值，真实深度需要SDK                      ║
╚═══════════════════════════════════════════════════════════╝
    """)
    
    # 1. 初始化相机
    camera = P100RCamera(camera_index=0)
    try:
        camera.initialize()
    except Exception as e:
        print(f"❌ 相机初始化失败: {e}")
        return
    
    # 2. 初始化SLAM检测器
    print("初始化SLAM避障系统...")
    slam = DepthSLAMObstacleDetector(
        depth_threshold_near=0.5,  # 近距离阈值 0.5m
        depth_threshold_far=5.0,   # 远距离阈值 5.0m
        obstacle_height_min=0.1,   # 最小障碍物高度 0.1m
        grid_resolution=0.05       # 5cm网格分辨率
    )
    print("✅ SLAM系统就绪\n")
    
    # 3. 选择深度估算方法
    print("选择深度估算方法:")
    print("  1 - brightness (基于亮度)")
    print("  2 - edge (基于边缘密度)")
    print("  3 - hybrid (混合方法，推荐)")
    
    choice = input("\n请选择 (1/2/3) [默认: 3]: ").strip() or "3"
    method_map = {'1': 'brightness', '2': 'edge', '3': 'hybrid'}
    depth_method = method_map.get(choice, 'hybrid')
    
    print(f"\n✅ 使用方法: {depth_method}")
    print("\n" + "=" * 60)
    print("开始实时SLAM避障测试")
    print("按 'q' 退出, 's' 截图, '1/2/3' 切换深度估算方法")
    print("=" * 60 + "\n")
    
    # 4. 主循环
    frame_count = 0
    start_time = time.time()
    screenshot_count = 0
    
    try:
        while True:
            # 读取彩色帧
            color_frame = camera.get_frame()
            if color_frame is None:
                print("[WARNING] 无法读取帧")
                break
            
            # 估算深度（mm转为m）
            depth_frame_mm = camera.estimate_depth(color_frame, method=depth_method)
            depth_frame = depth_frame_mm.astype(np.float32) / 1000.0  # 转换为米
            
            # SLAM处理
            obstacle_mask, info = slam.process_depth_frame(depth_frame, color_frame)
            
            # 提取信息
            navigable_zones = info.get('navigable_zones', [])
            direction = info.get('suggested_direction', 'unknown')
            
            # 可视化深度图
            depth_normalized = np.clip(depth_frame / 5.0, 0, 1)  # 归一化到0-1
            vis_depth = cv2.applyColorMap((depth_normalized * 255).astype(np.uint8), cv2.COLORMAP_JET)
            
            # 可视化障碍物
            vis_obstacles = cv2.cvtColor(obstacle_mask.astype(np.uint8) * 255, cv2.COLOR_GRAY2BGR)
            
            # 在障碍物图上绘制可导航区域
            for zone in navigable_zones:
                if 'centroid' in zone and 'area' in zone:
                    cx, cy = zone['centroid']
                    area = zone['area']
                    cv2.circle(vis_obstacles, (int(cx), int(cy)), int(np.sqrt(area)), (0, 255, 0), 2)
            
            # 计算FPS
            frame_count += 1
            if frame_count % 30 == 0:
                elapsed = time.time() - start_time
                fps = frame_count / elapsed
                print(f"[INFO] FPS: {fps:.1f} | 方向: {direction} | 可导航区域: {len(navigable_zones)}")
            
            # 显示
            # 缩放以适应屏幕（1920x1080太大）
            scale = 0.5
            color_small = cv2.resize(color_frame, None, fx=scale, fy=scale)
            depth_small = cv2.resize(vis_depth, None, fx=scale, fy=scale)
            obstacles_small = cv2.resize(vis_obstacles, None, fx=scale, fy=scale)
            
            # 添加标题
            cv2.putText(color_small, f"P100R Color ({depth_method})", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(depth_small, "Estimated Depth (NOT REAL!)", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.putText(obstacles_small, f"SLAM: {direction}", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            cv2.imshow('P100R Color', color_small)
            cv2.imshow('Depth (Estimated)', depth_small)
            cv2.imshow('SLAM Obstacles', obstacles_small)
            
            # 键盘控制
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('q'):
                print("\n用户退出")
                break
            elif key == ord('s'):
                screenshot_count += 1
                cv2.imwrite(f'color_{screenshot_count}.png', color_frame)
                cv2.imwrite(f'depth_{screenshot_count}.png', vis_depth)
                cv2.imwrite(f'obstacles_{screenshot_count}.png', vis_obstacles)
                print(f"📸 截图保存: *_{screenshot_count}.png")
            elif key == ord('1'):
                depth_method = 'brightness'
                print(f"切换到: {depth_method}")
            elif key == ord('2'):
                depth_method = 'edge'
                print(f"切换到: {depth_method}")
            elif key == ord('3'):
                depth_method = 'hybrid'
                print(f"切换到: {depth_method}")
    
    except KeyboardInterrupt:
        print("\n\n中断退出")
    
    finally:
        # 清理
        camera.release()
        cv2.destroyAllWindows()
        
        # 统计
        elapsed = time.time() - start_time
        avg_fps = frame_count / elapsed if elapsed > 0 else 0
        
        print("\n" + "=" * 60)
        print("测试完成!")
        print(f"  总帧数: {frame_count}")
        print(f"  运行时间: {elapsed:.1f}秒")
        print(f"  平均FPS: {avg_fps:.1f}")
        print("=" * 60)
        
        print("\n⚠️ 重要提示:")
        print("  当前使用的深度是从彩色图估算的，不是真实深度！")
        print("  要获取P100R的真实深度数据，需要:")
        print("    1. 从Berxel官网下载完整SDK")
        print("    2. 将 BerxelHawk.dll 和 .lib 复制到 libs/ 目录")
        print("    3. 编译 berxel_wrapper.cpp: python setup.py build_ext --inplace")
        print("    4. 使用 test_p100r_slam.py 运行真实深度测试")
        print("\n  参考文档: GET_SDK.md")


if __name__ == "__main__":
    main()
