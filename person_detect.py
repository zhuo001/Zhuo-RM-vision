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

# YOLO 推理参数配置
YOLO_CONF_THRESHOLD = 0.55  # 置信度阈值（提高以减少误检）
YOLO_IOU_THRESHOLD = 0.45   # NMS IOU阈值
YOLO_MAX_DETECTIONS = 50    # 最大检测数量

def is_valid_person(box, confidence, frame_height, frame_width):
    """
    检查检测框是否符合人形特征
    
    优化说明：
    1. 放宽尺寸限制以支持近距离检测
    2. 放宽宽高比以支持更多姿势（坐姿、侧身等）
    3. 提高置信度阈值以减少误检
    """
    x1, y1, x2, y2 = box
    width = x2 - x1
    height = y2 - y1
    
    if width <= 0 or height <= 0:
        return False
    
    # 计算宽高比
    aspect_ratio = height / width
    
    # 人的宽高比限制（放宽以支持更多姿势）
    MIN_ASPECT_RATIO = 0.5  # 允许横向姿势（躺、趴、近距离宽画面）
    MAX_ASPECT_RATIO = 5.0  # 允许站立瘦长身形
    
    if aspect_ratio < MIN_ASPECT_RATIO or aspect_ratio > MAX_ASPECT_RATIO:
        return False
    
    # 面积限制（相对于图像尺寸）
    area = width * height
    frame_area = frame_height * frame_width
    area_ratio = area / frame_area
    
    # 检测框不能太小（< 0.5%画面）或太大（> 95%画面）
    if area_ratio < 0.005 or area_ratio > 0.95:
        return False
    
    # 宽度和高度的绝对限制（1920x1080 分辨率下）
    # 最小尺寸：过滤掉太小的误检
    if width < 60 or height < 100:
        return False
    
    # 最大尺寸：允许近距离检测（几乎全屏）
    if width > 1850 or height > 1070:
        return False
        
    # 置信度阈值（提高以减少领带等误检）
    if confidence < 0.55:
        return False
    
    return True

def get_depth_at_point(depth_map, x, y, window_size=5):
    """
    获取指定点的深度值，使用周围区域的中位数以减少噪声
    
    Args:
        depth_map: 深度图（已缩放到彩色图尺寸）
        x, y: 中心点坐标
        window_size: 采样窗口大小（默认5x5）
    
    Returns:
        深度值（毫米），如果无效则返回 None
    """
    if depth_map is None:
        return None
    
    h, w = depth_map.shape[:2]
    
    # 计算采样区域
    half_win = window_size // 2
    y_min = max(0, y - half_win)
    y_max = min(h, y + half_win + 1)
    x_min = max(0, x - half_win)
    x_max = min(w, x + half_win + 1)
    
    # 提取区域
    region = depth_map[y_min:y_max, x_min:x_max]
    
    # 过滤掉无效值（0值）
    valid_depths = region[region > 0]
    
    if len(valid_depths) == 0:
        return None
    
    # 使用中位数减少噪声影响
    depth_value = np.median(valid_depths)
    
    # 合理性检查：P100R 深度转换系数是 1/17 mm
    # 有效范围：5000-136000 (对应 0.3m - 8m)
    if depth_value < 3000 or depth_value > 150000:
        return None
    
    return float(depth_value)

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
            depth_resized = None
            if depth is not None and depth.size > 0:
                try:
                    # P100R 深度图是 640x400，需要缩放到彩色图尺寸 1920x1080
                    depth_resized = cv2.resize(depth, (w, h), interpolation=cv2.INTER_NEAREST)
                    
                    # 将深度值映射到 0-255 范围的伪彩色（只考虑有效深度值）
                    valid_depth = depth_resized[depth_resized > 0]
                    
                    if len(valid_depth) > 0:
                        depth_min = np.percentile(valid_depth, 1)  # 使用1%分位数，避免极端值
                        depth_max = np.percentile(valid_depth, 99)  # 使用99%分位数
                        
                        if depth_max > depth_min:
                            # 归一化深度图
                            depth_norm = np.clip((depth_resized - depth_min) / (depth_max - depth_min), 0, 1)
                            depth_norm = (depth_norm * 255).astype(np.uint8)
                            
                            # 应用伪彩色
                            depth_color = cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)
                            
                            # 将无效区域（0值）显示为黑色
                            mask = depth_resized == 0
                            depth_color[mask] = [0, 0, 0]
                            
                            display[:, w:] = depth_color
                    else:
                        # 如果没有有效深度值，显示黑屏
                        display[:, w:] = 0
                        
                except Exception as e:
                    print(f"深度图处理错误: {e}")
                    display[:, w:] = 0

            try:
                # 运行目标检测（只检测人类，class 0）
                results = model(
                    frame,
                    conf=YOLO_CONF_THRESHOLD,
                    iou=YOLO_IOU_THRESHOLD,
                    max_det=YOLO_MAX_DETECTIONS,
                    classes=[0],  # 只检测人类
                    verbose=False  # 关闭详细输出
                )
                
                # 处理检测结果
                for result in results:
                    boxes = result.boxes
                    for box in boxes:
                        try:
                            # 获取类别ID（确保是人类）
                            class_id = int(box.cls[0])
                            if class_id != 0:  # 0 = person in COCO dataset
                                continue
                            
                            # 获取边界框坐标
                            x1, y1, x2, y2 = box.xyxy[0]
                            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                            
                            # 获取置信度
                            confidence = float(box.conf)
                            
                            # 检查是否是有效的人形检测
                            if not is_valid_person((x1, y1, x2, y2), confidence, h, w):
                                continue
                            
                            # 计算中心点坐标
                            center_x = int((x1 + x2) / 2)
                            center_y = int((y1 + y2) / 2)
                            
                            # 获取深度值（使用改进的中位数方法）
                            depth_value = get_depth_at_point(depth_resized, center_x, center_y, window_size=7)
                            
                            # 准备标签文本
                            label = f"Person {confidence:.2f}"
                            if depth_value is not None:
                                # P100R 深度转换：原始值需要除以17得到毫米，再转换为米
                                distance_m = depth_value / 17000.0
                                label += f" {distance_m:.2f}m"
                            else:
                                label += " (no depth)"
                            
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
