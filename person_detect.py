import cv2
import numpy as np
import time
import onnxruntime as ort

from berxel_camera import BerxelCamera

# 在启动时打印环境信息（方便诊断 OpenVINO / ONNXRuntime 可用性）
def _print_runtime_env():
    try:
        import onnxruntime as ort
        print('onnxruntime version:', ort.__version__)
        print('Available providers:', ort.get_available_providers())
    except Exception as e:
        print('onnxruntime not fully available:', e)

    try:
        import openvino as ov
        print('openvino version:', ov.__version__)
    except Exception as e:
        print('openvino not available:', e)

    # OpenCL ICD
    import os
    vendors_dir = '/etc/OpenCL/vendors'
    if os.path.isdir(vendors_dir):
        print('/etc/OpenCL/vendors:', os.listdir(vendors_dir))
    else:
        print('/etc/OpenCL/vendors not found')


# 打印一次环境信息
_print_runtime_env()

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

# 加载 ONNX Runtime 会话（AMD 780M 优化）
print("Initializing ONNX Runtime...")
# 优先尝试 ROCm（AMD GPU），fallback 到 CPU
available_providers = ort.get_available_providers()
print(f"Available ONNX Runtime providers: {available_providers}")

if 'ROCMExecutionProvider' in available_providers:
    providers = ['ROCMExecutionProvider', 'CPUExecutionProvider']
    print("✅ Using ROCMExecutionProvider (AMD GPU acceleration)")
elif 'CUDAExecutionProvider' in available_providers:
    providers = ['CUDAExecutionProvider', 'CPUExecutionProvider']
    print("⚠️ CUDA found but you have AMD 780M - using CPU instead")
    providers = ['CPUExecutionProvider']
else:
    providers = ['CPUExecutionProvider']
    print("ℹ️ Using CPUExecutionProvider (optimized for x86)")

# 加载 ONNX 模型
onnx_model_path = 'yolov8n.onnx'
session = ort.InferenceSession(onnx_model_path, providers=providers)

# 获取输入输出信息
input_name = session.get_inputs()[0].name
output_names = [o.name for o in session.get_outputs()]
print(f"Model input: {input_name}, outputs: {output_names}")

# YOLO 推理参数配置
YOLO_CONF_THRESHOLD = 0.40  # 置信度阈值（降低以检测更多目标）
YOLO_IOU_THRESHOLD = 0.45   # NMS IOU阈值
YOLO_MAX_DETECTIONS = 100   # 最大检测数量（增加以支持更多检测）

# 性能优化参数 - 针对 AMD 780M 优化（调整后：更频繁检测）
SKIP_FRAMES = 1  # 减少跳帧（1=每隔1帧检测，更流畅的检测）
YOLO_INPUT_SIZE = 416  # YOLO输入分辨率（从默认640降低到416以加速推理）
DISPLAY_SCALE = 0.5  # 显示窗口缩放比例（降低到50%以减少渲染负担）
TARGET_FPS = 30  # 目标帧率
DEPTH_PROCESS_INTERVAL = 3  # 每隔N帧处理一次深度图（深度图处理也很耗时）
# 深度平滑与可视化参数
DEPTH_EMA_ALPHA = 0.25  # EMA 平滑系数（0-1），越小平滑越强
DEPTH_VIS_KEEP_FRAMES = 5  # 当跳帧不处理深度时，保留上一次深度可视化的帧数

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
    
    # 人的宽高比限制（极度放宽以支持各种姿势和距离）
    MIN_ASPECT_RATIO = 0.3  # 允许横向姿势（躺、趴、近距离宽画面）
    MAX_ASPECT_RATIO = 8.0  # 允许站立瘦长身形，甚至垂直很长的检测
    
    if aspect_ratio < MIN_ASPECT_RATIO or aspect_ratio > MAX_ASPECT_RATIO:
        return False
    
    # 面积限制（相对于图像尺寸）- 放宽限制
    area = width * height
    frame_area = frame_height * frame_width
    area_ratio = area / frame_area
    
    # 检测框不能太小（< 0.2%画面）或太大（> 98%画面）- 大幅放宽
    if area_ratio < 0.002 or area_ratio > 0.98:
        return False
    
    # 宽度和高度的绝对限制（1920x1080 分辨率下）
    # 最小尺寸：降低阈值以检测更小的目标
    if width < 40 or height < 60:
        return False
    
    # 最大尺寸：完全放开，允许超大检测框（全屏人物）
    # 不设置上限，或者设置非常宽松的上限
    if width > 1900 or height > 1100:
        return False
        
    # 置信度阈值（降低以接受更多检测）
    if confidence < 0.45:  # 从0.55降低到0.45，允许更多检测
        return False
    
    return True

def run_onnx_inference(frame, session, input_name, input_size=416):
    """
    使用 ONNX Runtime 运行 YOLOv8 推理
    
    Args:
        frame: 输入图像 (BGR)
        session: ONNX Runtime 会话
        input_name: 输入节点名称
        input_size: 输入尺寸
    
    Returns:
        detections: (N, 6) numpy array [x1, y1, x2, y2, confidence, class_id]
    """
    # 预处理
    img = cv2.resize(frame, (input_size, input_size))
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img = img.transpose(2, 0, 1).astype(np.float32) / 255.0
    img = np.expand_dims(img, axis=0)
    
    # 推理
    outputs = session.run(None, {input_name: img})
    
    # YOLOv8 ONNX 输出格式：(1, 84, num_anchors)
    # 84 = 4 (bbox) + 80 (classes)
    predictions = outputs[0]  # (1, 84, num_anchors)
    predictions = predictions[0].T  # (num_anchors, 84)
    
    # 解析检测结果
    boxes = predictions[:, :4]  # (num_anchors, 4) - x_center, y_center, w, h
    scores = predictions[:, 4:]  # (num_anchors, 80) - class scores
    
    # 获取每个检测的最高类别分数和类别ID
    class_ids = np.argmax(scores, axis=1)
    confidences = np.max(scores, axis=1)
    
    # 过滤低置信度检测
    mask = confidences > YOLO_CONF_THRESHOLD
    boxes = boxes[mask]
    confidences = confidences[mask]
    class_ids = class_ids[mask]
    
    # 只保留人类检测（class 0）
    person_mask = class_ids == 0
    boxes = boxes[person_mask]
    confidences = confidences[person_mask]
    
    # 转换坐标格式：center format -> corner format
    x_center, y_center, width, height = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
    x1 = x_center - width / 2
    y1 = y_center - height / 2
    x2 = x_center + width / 2
    y2 = y_center + height / 2
    
    # NMS（非极大值抑制）
    if len(boxes) > 0:
        indices = cv2.dnn.NMSBoxes(
            boxes.tolist(),
            confidences.tolist(),
            YOLO_CONF_THRESHOLD,
            YOLO_IOU_THRESHOLD
        )
        
        if len(indices) > 0:
            if isinstance(indices, tuple):
                indices = list(indices)
            if isinstance(indices, list) and len(indices) > 0:
                indices = np.array(indices).flatten()
            x1 = x1[indices]
            y1 = y1[indices]
            x2 = x2[indices]
            y2 = y2[indices]
            confidences = confidences[indices]
            
            # 组合结果
            detections = np.column_stack([x1, y1, x2, y2, confidences])
            return detections
    
    return np.array([])

def get_depth_at_point(depth_map, x, y, window_size=5):
    """
    获取指定点的深度值，使用周围区域的中位数以减少噪声
    
    Args:
        depth_map: 深度图（已缩放到彩色图尺寸）
        x, y: 中心点坐标
        window_size: 采样窗口大小（默认5x5)
    
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
        
        # 性能优化变量
        frame_count = 0
        last_detections = []  # 缓存上一次的检测结果
        fps_start_time = time.time()
        fps_frame_count = 0
        current_fps = 0
        
        while True:
            loop_start = time.time()
            
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
            
            # 帧计数器
            frame_count += 1
            fps_frame_count += 1
            
            # 如果有深度图，处理并显示在右侧（优化：降低处理频率）
            depth_resized = None
            # 持久化和平滑相关变量（局部变量外提）
            if 'last_depth_color' not in globals():
                # 上一次用于显示的伪彩色图（用于未处理帧重用）
                globals()['last_depth_color'] = None
                globals()['last_depth_raw'] = None
                globals()['ema_depth_min'] = None
                globals()['ema_depth_max'] = None
                globals()['last_depth_keep_counter'] = 0

            if depth is not None and depth.size > 0 and frame_count % DEPTH_PROCESS_INTERVAL == 0:
                try:
                    # P100R 深度图是 640x400，需要缩放到彩色图尺寸 1920x1080
                    depth_resized = cv2.resize(depth, (w, h), interpolation=cv2.INTER_NEAREST)
                    
                    # 将深度值映射到 0-255 范围的伪彩色（只考虑有效深度值）
                    valid_depth = depth_resized[depth_resized > 0]
                    
                    if len(valid_depth) > 0:
                        depth_min = np.percentile(valid_depth, 1)  # 使用1%分位数，避免极端值
                        depth_max = np.percentile(valid_depth, 99)  # 使用99%分位数

                        # EMA 平滑 min/max，避免每帧大幅变动导致色彩闪烁
                        if globals()['ema_depth_min'] is None:
                            globals()['ema_depth_min'] = float(depth_min)
                        else:
                            globals()['ema_depth_min'] = (DEPTH_EMA_ALPHA * float(depth_min)
                                                           + (1 - DEPTH_EMA_ALPHA) * globals()['ema_depth_min'])

                        if globals()['ema_depth_max'] is None:
                            globals()['ema_depth_max'] = float(depth_max)
                        else:
                            globals()['ema_depth_max'] = (DEPTH_EMA_ALPHA * float(depth_max)
                                                           + (1 - DEPTH_EMA_ALPHA) * globals()['ema_depth_max'])

                        depth_min_s = globals()['ema_depth_min']
                        depth_max_s = globals()['ema_depth_max']

                        if depth_max_s > depth_min_s:
                            # 归一化深度图
                            depth_norm = np.clip((depth_resized - depth_min_s) / (depth_max_s - depth_min_s), 0, 1)
                            depth_norm = (depth_norm * 255).astype(np.uint8)

                            # 应用伪彩色
                            depth_color = cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)

                            # 将无效区域（0值）显示为黑色
                            mask = depth_resized == 0
                            depth_color[mask] = [0, 0, 0]

                            display[:, w:] = depth_color

                            # 更新持久化缓存
                            globals()['last_depth_color'] = depth_color.copy()
                            globals()['last_depth_raw'] = depth_resized.copy()
                            globals()['last_depth_keep_counter'] = 0
                    else:
                        # 如果没有有效深度值，显示黑屏
                        display[:, w:] = 0
                        
                except Exception as e:
                    print(f"深度图处理错误: {e}")
                    display[:, w:] = 0
            else:
                # 未在本帧处理深度：如果存在上次的持久化深度并且未超时，则重用它以避免闪烁
                last_color = globals().get('last_depth_color', None)
                last_raw = globals().get('last_depth_raw', None)
                if last_color is not None and globals().get('last_depth_keep_counter', 0) < DEPTH_VIS_KEEP_FRAMES:
                    display[:, w:] = last_color
                    globals()['last_depth_keep_counter'] += 1
                    # 将 depth_resized 设为持久化的原始深度，供后续检测读取深度使用
                    depth_resized = last_raw
                else:
                    # 没有可用缓存，就显示黑屏
                    display[:, w:] = 0

            # 性能优化：每隔N帧才进行一次YOLO检测
            try:
                if frame_count % (SKIP_FRAMES + 1) == 0:
                    # 使用 ONNX Runtime 运行推理
                    detections = run_onnx_inference(frame, session, input_name, YOLO_INPUT_SIZE)
                    
                    # 计算坐标缩放比例（从YOLO输入尺寸映射回原图）
                    scale_x = w / YOLO_INPUT_SIZE
                    scale_y = h / YOLO_INPUT_SIZE
                    
                    # 缓存检测结果
                    last_detections = []
                    for det in detections:
                        # det = [x1, y1, x2, y2, confidence]（相对于416x416）
                        x1 = int(det[0] * scale_x)
                        y1 = int(det[1] * scale_y)
                        x2 = int(det[2] * scale_x)
                        y2 = int(det[3] * scale_y)
                        confidence = float(det[4])
                        
                        # 检查是否是有效的人形检测
                        if is_valid_person((x1, y1, x2, y2), confidence, h, w):
                            last_detections.append((x1, y1, x2, y2, confidence))
                
                # 绘制检测结果（使用缓存的检测结果）
                for detection in last_detections:
                    x1, y1, x2, y2, confidence = detection
                    try:
                        # 计算中心点坐标
                        center_x = int((x1 + x2) / 2)
                        center_y = int((y1 + y2) / 2)
                        
                        # 获取深度值（使用改进的中位数方法）
                        depth_source = depth_resized if depth_resized is not None else globals().get('last_depth_raw', None)
                        depth_value = get_depth_at_point(depth_source, center_x, center_y, window_size=7)
                        
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
                
                # 计算并显示FPS
                if fps_frame_count >= 30:
                    elapsed = time.time() - fps_start_time
                    current_fps = fps_frame_count / elapsed
                    fps_start_time = time.time()
                    fps_frame_count = 0
                
                # 在画面上显示FPS
                cv2.putText(display, f"FPS: {current_fps:.1f}", (10, 30),
                          cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
                
                # 缩放显示窗口以减少显示负担
                display_h = int(h * DISPLAY_SCALE)
                display_w = int(w * 2 * DISPLAY_SCALE)
                display_resized = cv2.resize(display, (display_w, display_h))
                
                # 显示结果
                cv2.imshow('Person Detection with Depth', display_resized)
                
                # 按'q'键退出
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                
                # 帧率控制
                elapsed = time.time() - loop_start
                target_delay = 1.0 / TARGET_FPS
                if elapsed < target_delay:
                    time.sleep(target_delay - elapsed)
                    
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
