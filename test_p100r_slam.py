"""
Berxel P100R 相机测试 + SLAM 避障集成
测试相机连接、深度图获取，并集成到SLAM避障系统

作者: Zhuo
日期: 2025-10-15
"""

import cv2
import numpy as np
import time
from berxel_camera import BerxelCamera
from depth_slam_obstacle import DepthSLAMObstacleDetector


def test_camera_connection():
    """测试相机连接"""
    print("=" * 60)
    print("测试 Berxel P100R 相机连接")
    print("=" * 60)
    
    camera = BerxelCamera()
    
    print("\n[步骤 1/3] 初始化相机...")
    if not camera.initialize():
        print("[ERROR] 相机初始化失败！")
        print("请检查:")
        print("  1. P100R是否正确连接到USB端口")
        print("  2. Berxel驱动是否已安装")
        print("  3. berxel_wrapper扩展是否已编译")
        return None
    
    print("[OK] 相机初始化成功！")
    
    print("\n[步骤 2/3] 测试彩色图像获取...")
    color_frame = camera.get_frame()
    if color_frame is None:
        print("[ERROR] 无法获取彩色图像")
        camera.release()
        return None
    
    print(f"[OK] 彩色图像尺寸: {color_frame.shape}")
    
    print("\n[步骤 3/3] 测试深度图像获取...")
    depth_frame = camera.get_depth()
    if depth_frame is None:
        print("[ERROR] 无法获取深度图像")
        camera.release()
        return None
    
    print(f"[OK] 深度图像尺寸: {depth_frame.shape}")
    print(f"[OK] 深度范围: {np.min(depth_frame[depth_frame > 0]):.1f}mm - {np.max(depth_frame):.1f}mm")
    
    print("\n" + "=" * 60)
    print("✅ 相机测试通过！")
    print("=" * 60)
    
    return camera


def test_slam_with_camera(camera):
    """使用真实相机测试SLAM避障系统"""
    print("\n" + "=" * 60)
    print("启动 SLAM 避障系统 (使用 P100R 相机)")
    print("=" * 60)
    
    # 初始化SLAM检测器
    detector = DepthSLAMObstacleDetector(
        depth_threshold_near=500,   # 500mm = 0.5m
        depth_threshold_far=5000,   # 5000mm = 5m
        obstacle_height_min=100,    # 100mm = 0.1m
        grid_resolution=50          # 50mm = 5cm
    )
    
    print("\n[INFO] SLAM检测器初始化完成")
    print(f"  - 近距离阈值: {detector.depth_threshold_near}mm")
    print(f"  - 远距离阈值: {detector.depth_threshold_far}mm")
    print(f"  - 网格分辨率: {detector.grid_resolution}mm")
    print("\n[INFO] 按 'q' 退出, 's' 截图保存")
    print("=" * 60)
    
    frame_count = 0
    screenshot_count = 0
    
    try:
        while True:
            # 获取相机数据
            color_frame = camera.get_frame()
            depth_frame = camera.get_depth()
            
            if color_frame is None or depth_frame is None:
                print("[WARNING] 无法获取相机数据，跳过此帧")
                continue
            
            # 将深度从mm转换为m
            depth_in_meters = depth_frame.astype(np.float32) / 1000.0
            
            # 处理深度帧
            obstacle_mask, info = detector.process_depth_frame(
                depth_in_meters, 
                color_frame
            )
            
            # 可视化
            vis_image = detector.visualize(
                depth_in_meters, 
                obstacle_mask, 
                info, 
                color_frame
            )
            
            # 在可视化图像上添加相机信息
            cv2.putText(
                vis_image, 
                "Berxel P100R Camera", 
                (10, vis_image.shape[0] - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 
                0.6, 
                (0, 255, 255), 
                2
            )
            
            # 显示结果
            cv2.imshow('SLAM Obstacle Detection (P100R)', vis_image)
            cv2.imshow('Color Frame', color_frame)
            cv2.imshow('Obstacle Mask', obstacle_mask)
            
            # 每10帧打印一次统计信息
            frame_count += 1
            if frame_count % 10 == 0:
                print(f"[Frame {frame_count}] "
                      f"FPS: {1000/info['processing_time']:.1f} | "
                      f"Direction: {info['suggested_direction']} | "
                      f"Depth: {info['min_depth']:.2f}m-{info['max_depth']:.2f}m | "
                      f"Obstacles: {info['obstacle_count']}")
            
            # 按键处理
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                print("\n[INFO] 用户退出")
                break
            elif key == ord('s'):
                # 保存截图
                screenshot_count += 1
                filename = f"slam_screenshot_{screenshot_count}.png"
                cv2.imwrite(filename, vis_image)
                print(f"[INFO] 截图已保存: {filename}")
    
    except KeyboardInterrupt:
        print("\n[INFO] 用户中断 (Ctrl+C)")
    
    except Exception as e:
        print(f"\n[ERROR] 运行时错误: {str(e)}")
        import traceback
        traceback.print_exc()
    
    finally:
        cv2.destroyAllWindows()
        
        # 打印统计信息
        print("\n" + "=" * 60)
        print("运行统计:")
        print(f"  - 总帧数: {detector.frame_count}")
        if len(detector.processing_times) > 0:
            print(f"  - 平均处理时间: {np.mean(detector.processing_times):.3f}s")
            print(f"  - 平均FPS: {1.0/np.mean(detector.processing_times):.1f}")
        print("=" * 60)


def quick_camera_preview():
    """快速预览相机画面（不启用SLAM）"""
    print("=" * 60)
    print("Berxel P100R 快速预览模式")
    print("=" * 60)
    
    camera = BerxelCamera()
    
    if not camera.initialize():
        print("[ERROR] 相机初始化失败")
        return
    
    print("\n[INFO] 显示彩色和深度图像")
    print("[INFO] 按 'q' 退出")
    
    try:
        while True:
            color_frame = camera.get_frame()
            depth_frame = camera.get_depth()
            
            if color_frame is not None:
                cv2.imshow('P100R - Color', color_frame)
            
            if depth_frame is not None:
                # 深度图伪彩色显示
                depth_normalized = np.zeros_like(depth_frame, dtype=np.float32)
                cv2.normalize(depth_frame.astype(np.float32), depth_normalized, 0, 255, cv2.NORM_MINMAX)
                depth_colored = cv2.applyColorMap(
                    depth_normalized.astype(np.uint8),
                    cv2.COLORMAP_JET
                )
                cv2.imshow('P100R - Depth', depth_colored)
            
            key = cv2.waitKey(30) & 0xFF
            if key == ord('q'):
                break
    
    finally:
        cv2.destroyAllWindows()
        camera.release()


def main():
    """主函数"""
    import sys
    
    print("\n" + "=" * 60)
    print("Berxel P100R 相机测试与 SLAM 避障集成")
    print("=" * 60)
    print("\n请选择测试模式:")
    print("  1 - 快速预览相机画面")
    print("  2 - 完整SLAM避障测试")
    print("  3 - 仅测试相机连接")
    
    if len(sys.argv) > 1:
        choice = sys.argv[1]
    else:
        choice = input("\n请输入选项 (1/2/3) [默认: 2]: ").strip() or "2"
    
    if choice == "1":
        quick_camera_preview()
    
    elif choice == "2":
        # 完整SLAM测试
        camera = test_camera_connection()
        if camera is not None:
            input("\n按 Enter 键开始 SLAM 避障测试...")
            test_slam_with_camera(camera)
            camera.release()
    
    elif choice == "3":
        # 仅测试连接
        camera = test_camera_connection()
        if camera is not None:
            camera.release()
    
    else:
        print("[ERROR] 无效的选项")
        return
    
    print("\n程序结束\n")


if __name__ == "__main__":
    main()
