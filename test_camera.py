import cv2

def test_camera():
    # 尝试打开摄像头
    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        print("错误：无法打开摄像头")
        return
    
    print("摄像头已成功打开")
    print(f"分辨率：{int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}")
    print(f"FPS: {cap.get(cv2.CAP_PROP_FPS)}")
    
    try:
        while True:
            # 读取一帧
            ret, frame = cap.read()
            
            if not ret:
                print("错误：无法读取视频帧")
                break
                
            # 显示图像
            cv2.imshow('Camera Test', frame)
            
            # 按'q'键退出
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("测试完成")
                break
    
    finally:
        # 释放资源
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    test_camera()