"""
P100R相机简化测试 - 使用OpenCV直接访问
如果P100R支持UVC协议，可以直接用OpenCV访问

作者: Zhuo
日期: 2025-10-15
"""

import cv2
import numpy as np


def find_p100r_camera():
    """查找P100R相机（尝试多个索引）"""
    print("=" * 60)
    print("搜索 Berxel P100R 相机...")
    print("=" * 60)
    
    # 尝试索引0-5
    for idx in range(6):
        print(f"\n尝试相机索引 {idx}...")
        cap = cv2.VideoCapture(idx, cv2.CAP_DSHOW)  # 使用DirectShow后端（Windows）
        
        if cap.isOpened():
            # 读取一帧测试
            ret, frame = cap.read()
            if ret and frame is not None:
                print(f"✅ 在索引 {idx} 找到相机")
                print(f"   分辨率: {frame.shape[1]}x{frame.shape[0]}")
                
                # 获取相机属性
                fps = cap.get(cv2.CAP_PROP_FPS)
                backend = cap.getBackendName()
                print(f"   FPS: {fps}")
                print(f"   后端: {backend}")
                
                # 询问是否是P100R
                cv2.imshow(f'Camera {idx} Preview', frame)
                print(f"\n这是P100R相机吗？(按'y'确认, 'n'继续搜索, 'q'退出)")
                
                key = cv2.waitKey(3000) & 0xFF  # 3秒超时
                cv2.destroyAllWindows()
                
                if key == ord('y'):
                    print(f"\n✅ 已选择相机索引 {idx}")
                    return idx, cap
                elif key == ord('q'):
                    cap.release()
                    return None, None
                else:
                    cap.release()
                    continue
            else:
                cap.release()
        else:
            print(f"   索引 {idx} 无相机")
    
    print("\n❌ 未找到P100R相机")
    print("\n可能的原因:")
    print("  1. P100R不支持UVC协议（需要SDK）")
    print("  2. 相机驱动未正确安装")
    print("  3. 相机被其他程序占用")
    
    return None, None


def test_camera_simple():
    """简单相机测试"""
    idx, cap = find_p100r_camera()
    
    if cap is None:
        return
    
    print("\n" + "=" * 60)
    print("相机预览模式")
    print("按 'q' 退出, 's' 截图")
    print("=" * 60)
    
    screenshot_count = 0
    
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("[WARNING] 无法读取帧")
                break
            
            # 显示
            cv2.imshow('P100R Preview (OpenCV)', frame)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):
                screenshot_count += 1
                filename = f"p100r_test_{screenshot_count}.png"
                cv2.imwrite(filename, frame)
                print(f"截图保存: {filename}")
    
    finally:
        cap.release()
        cv2.destroyAllWindows()
        print("\n相机已释放")


def check_available_cameras():
    """列出所有可用相机"""
    print("=" * 60)
    print("检查所有可用相机")
    print("=" * 60)
    
    available = []
    for idx in range(10):
        cap = cv2.VideoCapture(idx)
        if cap.isOpened():
            ret, frame = cap.read()
            if ret:
                available.append({
                    'index': idx,
                    'resolution': f"{frame.shape[1]}x{frame.shape[0]}",
                    'backend': cap.getBackendName()
                })
            cap.release()
    
    if available:
        print(f"\n找到 {len(available)} 个相机:\n")
        for cam in available:
            print(f"  索引 {cam['index']}: {cam['resolution']} ({cam['backend']})")
    else:
        print("\n未找到任何相机")
    
    return available


if __name__ == "__main__":
    print("\n" + "=" * 60)
    print("P100R 相机简化测试工具")
    print("=" * 60)
    print("\n选项:")
    print("  1 - 查找并测试P100R")
    print("  2 - 列出所有相机")
    
    choice = input("\n请选择 (1/2) [默认: 1]: ").strip() or "1"
    
    if choice == "1":
        test_camera_simple()
    elif choice == "2":
        check_available_cameras()
    else:
        print("无效选项")
