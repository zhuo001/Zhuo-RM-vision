"""
P100R相机修复测试 - 解决绿色噪声问题
正确配置相机参数和色彩空间

作者: Zhuo
日期: 2025-10-15
"""

import cv2
import numpy as np
import time


def test_camera_with_proper_settings(camera_index=0):
    """使用正确的参数配置测试相机"""
    print("=" * 60)
    print(f"测试相机索引 {camera_index}")
    print("=" * 60)
    
    # 使用DirectShow后端（Windows推荐）
    cap = cv2.VideoCapture(camera_index, cv2.CAP_DSHOW)
    
    if not cap.isOpened():
        print(f"❌ 无法打开相机索引 {camera_index}")
        return False
    
    # 设置常见分辨率（P100R彩色图可能是1920x1080或640x480）
    resolutions = [
        (1920, 1080),
        (1280, 720),
        (640, 480),
        (640, 400),  # P100R深度图分辨率
    ]
    
    print("\n尝试设置分辨率...")
    for width, height in resolutions:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        
        actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        
        if actual_width == width and actual_height == height:
            print(f"✅ 成功设置分辨率: {width}x{height}")
            break
        else:
            print(f"   尝试 {width}x{height} -> 实际 {actual_width}x{actual_height}")
    
    # 设置FPS
    cap.set(cv2.CAP_PROP_FPS, 30)
    
    # 设置自动曝光
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)  # 3 = 自动模式
    
    # 设置FOURCC格式（尝试MJPG以获得更好的质量）
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
    
    # 等待相机初始化
    print("\n等待相机初始化（丢弃前几帧）...")
    for i in range(10):
        cap.read()
        time.sleep(0.1)
    
    # 读取并检查帧
    print("\n读取测试帧...")
    ret, frame = cap.read()
    
    if not ret or frame is None:
        print("❌ 无法读取帧")
        cap.release()
        return False
    
    print(f"✅ 成功读取帧:")
    print(f"   形状: {frame.shape}")
    print(f"   类型: {frame.dtype}")
    print(f"   值范围: [{frame.min()}, {frame.max()}]")
    print(f"   均值: R={frame[:,:,2].mean():.1f} G={frame[:,:,1].mean():.1f} B={frame[:,:,0].mean():.1f}")
    
    # 检查是否是绿色噪声
    mean_g = frame[:,:,1].mean()
    mean_r = frame[:,:,2].mean()
    mean_b = frame[:,:,0].mean()
    
    if mean_g > 200 and mean_r < 100 and mean_b < 100:
        print("\n⚠️ 警告: 检测到绿色噪声！")
        print("   可能原因:")
        print("   1. 相机格式不支持（需要SDK）")
        print("   2. 色彩空间转换错误")
        print("   3. 相机未正确初始化")
        
        # 尝试色彩空间转换
        print("\n尝试YUV色彩空间转换...")
        try:
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_YUV2BGR_YUYV)
            print("   尝试 YUV2BGR_YUYV 转换")
            cv2.imshow('YUV Conversion', frame_rgb)
        except:
            print("   转换失败")
    
    # 显示原始帧
    cv2.imshow(f'Camera {camera_index} - Original', frame)
    
    # 显示每个通道
    b, g, r = cv2.split(frame)
    cv2.imshow(f'Camera {camera_index} - Blue Channel', b)
    cv2.imshow(f'Camera {camera_index} - Green Channel', g)
    cv2.imshow(f'Camera {camera_index} - Red Channel', r)
    
    print("\n按任意键继续测试，或按 'q' 退出")
    key = cv2.waitKey(0)
    
    if key != ord('q'):
        # 实时预览
        print("\n开始实时预览...")
        print("按 'q' 退出")
        
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            
            # 添加信息叠加
            info_text = f"Index: {camera_index} | FPS: {cap.get(cv2.CAP_PROP_FPS):.1f}"
            cv2.putText(frame, info_text, (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            cv2.imshow(f'Camera {camera_index} Live', frame)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    
    cap.release()
    cv2.destroyAllWindows()
    return True


def test_all_available_cameras():
    """测试所有可用的相机"""
    print("\n" + "=" * 60)
    print("扫描所有相机...")
    print("=" * 60)
    
    available_cameras = []
    
    for idx in range(5):
        print(f"\n检查索引 {idx}...")
        cap = cv2.VideoCapture(idx, cv2.CAP_DSHOW)
        
        if cap.isOpened():
            # 等待初始化
            for _ in range(5):
                cap.read()
            
            ret, frame = cap.read()
            if ret and frame is not None:
                width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                fps = cap.get(cv2.CAP_PROP_FPS)
                
                # 检查是否是绿色噪声
                mean_g = frame[:,:,1].mean()
                is_noisy = mean_g > 200
                
                available_cameras.append({
                    'index': idx,
                    'resolution': f"{width}x{height}",
                    'fps': fps,
                    'is_noisy': is_noisy
                })
                
                print(f"   ✅ 相机可用: {width}x{height} @ {fps}fps")
                if is_noisy:
                    print(f"      ⚠️ 检测到绿色噪声")
            
            cap.release()
        else:
            print(f"   ❌ 无相机")
    
    return available_cameras


def main():
    print("""
╔═══════════════════════════════════════════════════════════╗
║         P100R 相机修复测试工具                            ║
║                                                           ║
║  目的: 解决绿色噪声问题，正确配置相机参数                ║
╚═══════════════════════════════════════════════════════════╝
    """)
    
    # 1. 扫描所有相机
    cameras = test_all_available_cameras()
    
    if not cameras:
        print("\n❌ 未找到任何相机")
        print("\n可能的解决方案:")
        print("  1. 检查P100R是否正确连接")
        print("  2. 检查设备管理器中的相机驱动")
        print("  3. 重新插拔USB连接")
        print("  4. 确认相机未被其他程序占用")
        return
    
    print("\n" + "=" * 60)
    print(f"找到 {len(cameras)} 个相机:")
    for cam in cameras:
        status = "⚠️ 噪声" if cam['is_noisy'] else "✅ 正常"
        print(f"  [{cam['index']}] {cam['resolution']} @ {cam['fps']}fps - {status}")
    
    print("=" * 60)
    
    # 2. 选择相机进行详细测试
    print("\n请选择要测试的相机索引 (或按Enter使用索引0):")
    choice = input("索引: ").strip()
    
    try:
        idx = int(choice) if choice else 0
    except ValueError:
        idx = 0
    
    test_camera_with_proper_settings(idx)
    
    print("\n" + "=" * 60)
    print("测试完成！")
    print("\n💡 如果仍然显示绿色噪声:")
    print("  → P100R需要使用官方SDK才能正确读取图像")
    print("  → 请参考 GET_SDK.md 获取SDK库文件")
    print("=" * 60)


if __name__ == "__main__":
    main()
