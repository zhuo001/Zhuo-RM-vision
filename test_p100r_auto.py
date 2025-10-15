"""
P100R相机自动测试 + SLAM避障集成
自动使用索引0的相机（假设为P100R）

作者: Zhuo
日期: 2025-10-15
"""

import cv2
import numpy as np
import time
from depth_slam_obstacle import DepthSLAMObstacleDetector


def test_slam_with_uvc_camera():
    """使用UVC相机测试SLAM避障（无深度信息）"""
    print("=" * 60)
    print("P100R 彩色相机测试 + 模拟深度SLAM")
    print("=" * 60)
    
    # 打开相机
    print("\n[INFO] 正在打开相机（索引0）...")
    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    
    if not cap.isOpened():
        print("[ERROR] 无法打开相机")
        return
    
    # 设置分辨率
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    # 读取一帧验证
    ret, frame = cap.read()
    if not ret:
        print("[ERROR] 无法读取相机帧")
        cap.release()
        return
    
    print(f"[OK] 相机已打开")
    print(f"[INFO] 分辨率: {frame.shape[1]}x{frame.shape[0]}")
    
    # 初始化SLAM检测器
    detector = DepthSLAMObstacleDetector(
        depth_threshold_near=0.5,   # 0.5m
        depth_threshold_far=5.0,    # 5m
        obstacle_height_min=0.1,    # 0.1m
        grid_resolution=0.05        # 5cm
    )
    
    print("\n[INFO] SLAM检测器初始化完成")
    print("[INFO] 模式: 彩色相机 + 模拟深度（从灰度估计）")
    print("[INFO] 按 'q' 退出, 's' 截图, 'd' 切换深度估计")
    print("=" * 60)
    
    frame_count = 0
    screenshot_count = 0
    depth_mode = 1  # 1=从亮度, 2=从边缘, 3=固定深度
    
    try:
        while True:
            ret, color_frame = cap.read()
            if not ret:
                print("[WARNING] 读取帧失败")
                continue
            
            # 从彩色图像估计深度（多种方法）
            if depth_mode == 1:
                # 方法1: 从亮度估计（亮的近，暗的远）
                gray = cv2.cvtColor(color_frame, cv2.COLOR_BGR2GRAY)
                # 反转：亮的（255）->近(0.5m), 暗的（0）->远(5m)
                depth_estimated = 5.0 - (gray.astype(np.float32) / 255.0) * 4.5
                mode_text = "Depth: Brightness-based"
            
            elif depth_mode == 2:
                # 方法2: 从边缘密度估计（边缘多的近，少的远）
                gray = cv2.cvtColor(color_frame, cv2.COLOR_BGR2GRAY)
                edges = cv2.Canny(gray, 50, 150)
                blur_edges = cv2.GaussianBlur(edges.astype(np.float32), (21, 21), 0)
                depth_estimated = 5.0 - (blur_edges / 255.0) * 4.5
                mode_text = "Depth: Edge-based"
            
            else:
                # 方法3: 固定中心近外围远
                h, w = color_frame.shape[:2]
                y, x = np.ogrid[:h, :w]
                center_dist = np.sqrt((x - w/2)**2 + (y - h/2)**2)
                max_dist = np.sqrt((w/2)**2 + (h/2)**2)
                depth_estimated = 0.5 + (center_dist / max_dist) * 4.5
                mode_text = "Depth: Radial"
            
            # 处理深度帧
            obstacle_mask, info = detector.process_depth_frame(
                depth_estimated,
                color_frame
            )
            
            # 可视化
            vis_image = detector.visualize(
                depth_estimated,
                obstacle_mask,
                info,
                color_frame
            )
            
            # 添加模式信息
            cv2.putText(
                vis_image,
                mode_text,
                (10, vis_image.shape[0] - 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 255),
                2
            )
            cv2.putText(
                vis_image,
                "P100R Color Camera (Simulated Depth)",
                (10, vis_image.shape[0] - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 255),
                2
            )
            
            # 显示
            cv2.imshow('P100R SLAM Test (Simulated Depth)', vis_image)
            cv2.imshow('P100R Color', color_frame)
            cv2.imshow('Obstacle Mask', obstacle_mask)
            
            # 每30帧打印统计
            frame_count += 1
            if frame_count % 30 == 0:
                print(f"[Frame {frame_count}] "
                      f"FPS: {1000/info['processing_time']:.1f} | "
                      f"Direction: {info['suggested_direction']} | "
                      f"Mode: {depth_mode}")
            
            # 按键处理
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                print("\n[INFO] 用户退出")
                break
            elif key == ord('s'):
                screenshot_count += 1
                filename = f"p100r_slam_{screenshot_count}.png"
                cv2.imwrite(filename, vis_image)
                print(f"[INFO] 截图保存: {filename}")
            elif key == ord('d'):
                depth_mode = (depth_mode % 3) + 1
                print(f"[INFO] 切换到深度模式 {depth_mode}")
    
    except KeyboardInterrupt:
        print("\n[INFO] 用户中断 (Ctrl+C)")
    
    except Exception as e:
        print(f"\n[ERROR] 运行时错误: {str(e)}")
        import traceback
        traceback.print_exc()
    
    finally:
        cap.release()
        cv2.destroyAllWindows()
        
        print("\n" + "=" * 60)
        print("运行统计:")
        print(f"  - 总帧数: {detector.frame_count}")
        if len(detector.processing_times) > 0:
            print(f"  - 平均处理时间: {np.mean(detector.processing_times):.3f}s")
            print(f"  - 平均FPS: {1.0/np.mean(detector.processing_times):.1f}")
        print("=" * 60)


def simple_camera_test():
    """简单相机预览"""
    print("=" * 60)
    print("P100R 简单预览测试")
    print("=" * 60)
    
    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    
    if not cap.isOpened():
        print("[ERROR] 无法打开相机")
        return
    
    print("\n[INFO] 相机已打开，按 'q' 退出")
    
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            
            cv2.imshow('P100R Color Camera', frame)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    print("\n" + "=" * 60)
    print("P100R 相机测试工具")
    print("=" * 60)
    print("\n测试模式:")
    print("  1 - 简单预览（仅彩色）")
    print("  2 - SLAM避障测试（模拟深度）")
    
    import sys
    if len(sys.argv) > 1:
        choice = sys.argv[1]
    else:
        choice = input("\n请选择模式 (1/2) [默认: 2]: ").strip() or "2"
    
    if choice == "1":
        simple_camera_test()
    elif choice == "2":
        test_slam_with_uvc_camera()
    else:
        print("[ERROR] 无效选项")
