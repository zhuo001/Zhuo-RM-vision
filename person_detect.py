from ultralytics import YOLO
import cv2
import numpy as np

from berxel_camera import BerxelCamera

def initialize_camera():
    try:
        # 初始化Berxel相机
        camera = BerxelCamera()
        if not camera.initialize():
            raise Exception("无法初始化Berxel相机")
        return camera
    except Exception as e:
        print(f"相机初始化失败: {e}")
        raise

# 加载YOLOv8n模型
model = YOLO('yolov8n.pt')

# 设置推理参数
model.conf = 0.45  # 提高置信度阈值
model.iou = 0.45   # NMS IOU阈值
model.max_det = 50 # 最大检测数量
model.classes = [0]  # 只检测人类类别

def is_valid_person(box, confidence):
    """检查检测框是否符合人形特征"""
    x1, y1, x2, y2 = box
    width = x2 - x1
    height = y2 - y1
    
    if width == 0 or height == 0:
        return False
    
    # 计算宽高比
    aspect_ratio = height / width
    
    # 人的宽高比限制（更严格）
    MIN_ASPECT_RATIO = 1.8  # 提高最小宽高比
    MAX_ASPECT_RATIO = 3.5  # 降低最大宽高比
    
    # 检查宽高比
    if aspect_ratio < MIN_ASPECT_RATIO or aspect_ratio > MAX_ASPECT_RATIO:
        return False
    
    # 面积限制
    area = width * height
    MIN_AREA = 10000  # 最小面积（像素）
    MAX_AREA = 300000  # 最大面积（像素）
    if area < MIN_AREA or area > MAX_AREA:
        return False
        
    # 位置检查（避免检测框过于靠近图像边缘）
    EDGE_MARGIN = 10  # 边缘余量（像素）
    if x1 < EDGE_MARGIN or y1 < EDGE_MARGIN:
        return False
        
    # 置信度阈值（动态调整）
    if confidence < 0.65:  # 提高置信度要求
        return False
    
    return True

def main():
    cap = None
    try:
        # 初始化相机
        cap = initialize_camera()
        
        while True:
            # 获取彩色图像和深度图
            frame = cap.get_frame()
            depth = cap.get_depth()
            
            if frame is None:
                print("无法获取图像帧")
                continue
            
            # 创建空白画布，用于并排显示
            h, w = frame.shape[:2]
            display = np.zeros((h, w*2, 3), dtype=np.uint8)
            
            # 将彩色图像放在左侧
            display[:, :w] = frame
            
            # 如果有深度图，处理并显示在右侧
            if depth is not None:
                # 将深度值映射到 0-255 范围的伪彩色
                depth_min = np.min(depth)
                depth_max = np.max(depth)
                if depth_max > depth_min:
                    depth_norm = ((depth - depth_min) * 255 / (depth_max - depth_min)).astype(np.uint8)
                    depth_color = cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)
                    display[:, w:] = depth_color

            try:
                # 运行目标检测
                results = model(frame)
                
                # 处理检测结果
                for result in results:
                    boxes = result.boxes
                    for box in boxes:
                        try:
                            # 获取边界框坐标
                            x1, y1, x2, y2 = box.xyxy[0]
                            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                            
                            # 获取置信度
                            confidence = float(box.conf)
                            
                            # 检查是否是有效的人形检测
                            if not is_valid_person((x1, y1, x2, y2), confidence):
                                continue
                            
                            # 计算中心点坐标
                            center_x = int((x1 + x2) / 2)
                            center_y = int((y1 + y2) / 2)
                            
                            # 获取深度值
                            depth_value = None
                            if depth is not None:
                                depth_value = depth[center_y, center_x]
                            
                            # 准备标签文本
                            label = f"Person {confidence:.2f}"
                            if depth_value is not None:
                                label += f" {depth_value/1000:.2f}m"
                            
                            # 在左侧彩色图像中绘制
                            cv2.rectangle(display, (x1, y1), (x2, y2), (0, 255, 0), 2)
                            cv2.putText(display, label, (x1, y1 - 10),
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                            cv2.drawMarker(display, (center_x, center_y),
                                         (0, 0, 255), cv2.MARKER_CROSS, 10, 2)
                            
                            # 在右侧深度图中也绘制相同的检测框
                            cv2.rectangle(display, (w+x1, y1), (w+x2, y2), (0, 255, 0), 2)
                            cv2.putText(display, label, (w+x1, y1 - 10),
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                            cv2.drawMarker(display, (w+center_x, center_y),
                                         (0, 0, 255), cv2.MARKER_CROSS, 10, 2)
                        except Exception as e:
                            print(f"处理检测框时发生错误: {e}")
                            continue
                
                # 显示结果
                cv2.imshow('Person Detection with Depth', display)
                
                # 按'q'键退出
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                    
            except Exception as e:
                print(f"YOLO检测过程中发生错误: {e}")
                continue
                
    except Exception as e:
        print(f"发生错误: {e}")
        
    finally:
        # 清理资源
        cv2.destroyAllWindows()
        if cap:
            cap.release()

if __name__ == "__main__":
    main()