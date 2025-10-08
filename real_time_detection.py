




























































































import cv2
import numpy as np
from ultralytics import YOLO

def initialize_camera():
    # 初始化普通USB摄像头
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        raise Exception("Cannot open camera")
    return cap

def main():
    # 初始化YOLOv8模型
    model = YOLO('yolov8n.pt')
    
    # 初始化摄像头
    try:
        cap = initialize_camera()
    except Exception as e:
        print(f"Error initializing camera: {e}")
        return

    try:
        while True:
            # 读取一帧
            ret, frame = cap.read()
            if not ret:
                print("Can't receive frame")
                break
            
            # 运行目标检测
            results = model(frame)
            
            # 处理检测结果
            for result in results:
                boxes = result.boxes
                for box in boxes:
                    # 获取边界框坐标
                    x1, y1, x2, y2 = box.xyxy[0]
                    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                    
                    # 绘制边界框和标签
                    confidence = float(box.conf)
                    class_id = int(box.cls)
                    label = f"{model.names[class_id]} {confidence:.2f}"
                    
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(frame, label, (x1, y1 - 10),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # 显示结果
            cv2.imshow("Real-time Detection", frame)
            
            # 按'q'退出
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
    finally:
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()