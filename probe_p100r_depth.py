"""
探测P100R相机的所有可用视频流格式
某些3D相机会通过多个索引或特殊FOURCC码提供深度流
"""
import cv2
import numpy as np

def probe_camera_streams():
    """探测相机的多个视频流"""
    print("=" * 60)
    print("探测P100R相机的所有视频流...")
    print("=" * 60)
    
    # 尝试打开多个索引
    for index in range(5):
        print(f"\n🔍 尝试索引 {index}...")
        cap = cv2.VideoCapture(index)
        
        if not cap.isOpened():
            print(f"   ❌ 索引 {index} 无法打开")
            continue
            
        # 获取基本信息
        width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = cap.get(cv2.CAP_PROP_FPS)
        fourcc = int(cap.get(cv2.CAP_PROP_FOURCC))
        fourcc_str = "".join([chr((fourcc >> 8 * i) & 0xFF) for i in range(4)])
        
        print(f"   ✅ 索引 {index} 可用:")
        print(f"      分辨率: {width}x{height}")
        print(f"      帧率: {fps}")
        print(f"      FOURCC: {fourcc_str} ({fourcc})")
        
        # 读取一帧测试
        ret, frame = cap.read()
        if ret:
            print(f"      帧形状: {frame.shape}")
            print(f"      数据类型: {frame.dtype}")
            print(f"      值范围: [{frame.min()}, {frame.max()}]")
            
            # 显示预览
            preview = cv2.resize(frame, (320, 240))
            cv2.imshow(f"Index {index} - {width}x{height}", preview)
        else:
            print(f"      ❌ 无法读取帧")
        
        cap.release()
    
    print("\n" + "=" * 60)
    print("按任意键继续...")
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def try_depth_formats():
    """尝试不同的格式读取深度"""
    print("\n" + "=" * 60)
    print("尝试不同格式读取深度流...")
    print("=" * 60)
    
    # P100R通常是索引0
    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        print("❌ 无法打开相机")
        return
    
    # 尝试设置不同的FOURCC格式
    formats = [
        ('YUYV', cv2.VideoWriter_fourcc('Y', 'U', 'Y', 'V')),
        ('MJPG', cv2.VideoWriter_fourcc('M', 'J', 'P', 'G')),
        ('GREY', cv2.VideoWriter_fourcc('G', 'R', 'E', 'Y')),
        ('Y16 ', cv2.VideoWriter_fourcc('Y', '1', '6', ' ')),  # 16位灰度
        ('RGBD', cv2.VideoWriter_fourcc('R', 'G', 'B', 'D')),
    ]
    
    for name, fourcc in formats:
        print(f"\n🔍 尝试格式 {name}...")
        cap.set(cv2.CAP_PROP_FOURCC, fourcc)
        
        ret, frame = cap.read()
        if ret:
            print(f"   ✅ 成功读取:")
            print(f"      形状: {frame.shape}")
            print(f"      类型: {frame.dtype}")
            print(f"      范围: [{frame.min()}, {frame.max()}]")
        else:
            print(f"   ❌ 无法读取")
    
    cap.release()

def check_depth_in_channels():
    """检查RGB通道中是否有深度信息"""
    print("\n" + "=" * 60)
    print("检查RGB通道中的深度信息...")
    print("=" * 60)
    
    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        print("❌ 无法打开相机")
        return
    
    ret, frame = cap.read()
    cap.release()
    
    if not ret:
        print("❌ 无法读取帧")
        return
    
    # 分析每个通道
    b, g, r = cv2.split(frame)
    
    print("\n📊 通道分析:")
    for i, (channel, name) in enumerate([(b, 'Blue'), (g, 'Green'), (r, 'Red')]):
        print(f"\n{name} 通道:")
        print(f"  均值: {channel.mean():.2f}")
        print(f"  标准差: {channel.std():.2f}")
        print(f"  最小值: {channel.min()}")
        print(f"  最大值: {channel.max()}")
        print(f"  唯一值数量: {len(np.unique(channel))}")
        
        # 显示直方图
        hist = cv2.calcHist([channel], [0], None, [256], [0, 256])
        print(f"  直方图峰值位置: {np.argmax(hist)}")

def main():
    print("""
╔═══════════════════════════════════════════════════════════╗
║     P100R 深度相机流格式探测工具                          ║
║                                                           ║
║  目的: 尝试通过OpenCV直接读取深度流（无SDK）              ║
║  注意: P100R可能需要SDK才能正确解析深度数据               ║
╚═══════════════════════════════════════════════════════════╝
    """)
    
    try:
        # 1. 探测所有视频流索引
        probe_camera_streams()
        
        # 2. 尝试不同格式
        try_depth_formats()
        
        # 3. 检查RGB通道
        check_depth_in_channels()
        
        print("\n" + "=" * 60)
        print("✅ 探测完成！")
        print("\n💡 结论:")
        print("   如果没有找到深度流，说明P100R确实需要SDK")
        print("   请参考 GET_SDK.md 获取完整SDK")
        print("=" * 60)
        
    except Exception as e:
        print(f"\n❌ 错误: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()
