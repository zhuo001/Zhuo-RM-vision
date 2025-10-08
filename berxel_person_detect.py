import cv2
import numpy as np
import os
import sys
from ultralytics import YOLO

# 添加 Berxel SDK 路径
berxel_lib_path = "/home/zhuo-skadi/Documents/ros2-robt/libs"
sys.path.append(berxel_lib_path)

try:
    import berxel
except ImportError as e:
    print(f"错误：无法导入 berxel SDK: {e}")
    print(f"请确认 SDK 路径是否正确: {berxel_lib_path}")
    sys.exit(1)
import berxel
import time

def initialize_camera():
    try:
        # 初始化 Berxel 相机
        device = berxel.Device()
        device.initialize()
        
        # 配置数据流
        device.start_streams()
        print("相机初始化成功！")
        return device
    except Exception as e:
        print(f"相机初始化失败: {e}")
        raise

def main():
    # 初始化 YOLOv8 模型，只检测人体
    model = YOLO('yolov8n.pt')
    model.classes = [0]  # 只检测人类类别
    
    try:
        # 初始化相机
        device = initialize_camera()
        
        while True:
            # 获取彩色图像和深度图像
            color_frame = device.get_color_frame()
            depth_frame = device.get_depth_frame()
            
            if color_frame is None or depth_frame is None:
                print("无法获取图像帧")
                continue
            
            # 转换为numpy数组
            color_image = np.array(color_frame.get_data())
            depth_image = np.array(depth_frame.get_data())
            
            # 运行目标检测
            results = model(color_image)
            
            # 处理检测结果
            for result in results:
                boxes = result.boxes
                for box in boxes:
                    # 获取边界框坐标
                    x1, y1, x2, y2 = box.xyxy[0]
                    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                    
                    # 计算中心点坐标
                    center_x = int((x1 + x2) / 2)
                    center_y = int((y1 + y2) / 2)
                    
                    # 获取中心点的深度值（单位：毫米）
                    depth_value = depth_image[center_y, center_x]
                    
                    # 计算目标的实际尺寸（使用深度信息）
                    target_width = abs(x2 - x1)
                    target_height = abs(y2 - y1)
                    
                    # 绘制边界框和信息
                    confidence = float(box.conf)
                    label = f"Person {confidence:.2f} Depth: {depth_value}mm"
                    
                    cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(color_image, label, (x1, y1 - 10),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    
                    # 在中心点绘制十字标记
                    cv2.drawMarker(color_image, (center_x, center_y), 
                                 (0, 0, 255), cv2.MARKER_CROSS, 10, 2)
            
            # 显示结果
            cv2.imshow('Person Detection', color_image)
            
            # 显示深度图像（归一化显示）
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), 
                                             cv2.COLORMAP_JET)
            cv2.imshow('Depth View', depth_colormap)
            
            # 按'q'键退出
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
    except Exception as e:
        print(f"发生错误: {e}")
        
    finally:
        # 清理资源
        if 'device' in locals():
            device.stop_streams()
            device.deinitialize()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()