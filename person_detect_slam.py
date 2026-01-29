#!/usr/bin/env python3
"""
视觉目标追踪系统
基于 YOLOv12 + Berxel P100R 深度相机

功能:
- 实时人员检测与跟踪 (YOLOv12 ONNX)
- 深度测量与距离标注
- 目标位置输出 (供 Nav2 导航使用)
- 单窗口可视化界面

架构:
- 本脚本: 视觉追踪 (P100R + YOLOv12) → 发布目标位置
- Nav2: 接收目标位置 + LiDAR融合 (Mid70 + L2) → 导航控制

作者: Zhuo RM Team
日期: 2026-01-29
"""

import cv2
import numpy as np
import time
import onnxruntime as ort
import sys

from berxel_camera import BerxelCamera

# ============== 环境信息 ==============
def _print_runtime_env():
    try:
        import onnxruntime as ort
        print('onnxruntime version:', ort.__version__)
        print('Available providers:', ort.get_available_providers())
    except Exception as e:
        print('onnxruntime not fully available:', e)

_print_runtime_env()

# ============== ONNX Runtime 初始化 ==============
print("Initializing ONNX Runtime...")
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
onnx_model_path = 'yolo12n.onnx'
session = ort.InferenceSession(onnx_model_path, providers=providers)

input_name = session.get_inputs()[0].name
output_names = [o.name for o in session.get_outputs()]
print(f"Model input: {input_name}, outputs: {output_names}")

# ============== 参数配置 ==============
# YOLO 参数
YOLO_CONF_THRESHOLD = 0.40
YOLO_IOU_THRESHOLD = 0.45
YOLO_INPUT_SIZE = 416

# 性能参数
SKIP_FRAMES = 1
DISPLAY_SCALE = 0.6
TARGET_FPS = 30

# 深度参数
DEPTH_WINDOW_SIZE = 7
DEPTH_MIN_VALID = 3000      # 最小有效深度 (mm)
DEPTH_MAX_VALID = 150000    # 最大有效深度 (mm)
DEPTH_SCALE = 17000.0       # P100R 深度转换系数


# ============== 辅助函数 ==============
def initialize_camera():
    """初始化 Berxel 相机"""
    try:
        camera = BerxelCamera()
        if not camera.initialize():
            raise Exception("无法初始化Berxel相机")
        return camera
    except Exception as e:
        print(f"相机初始化失败: {e}")
        raise


def is_valid_person(box, confidence, frame_height, frame_width):
    """检查检测框是否符合人形特征"""
    x1, y1, x2, y2 = box
    width = x2 - x1
    height = y2 - y1
    
    if width <= 0 or height <= 0:
        return False
    
    # 宽高比检查 (人形通常是纵向的)
    aspect_ratio = height / width
    if aspect_ratio < 0.3 or aspect_ratio > 8.0:
        return False
    
    # 面积比例检查
    area = width * height
    frame_area = frame_height * frame_width
    area_ratio = area / frame_area
    if area_ratio < 0.002 or area_ratio > 0.98:
        return False
    
    # 最小尺寸检查
    if width < 40 or height < 60:
        return False
    
    # 最大尺寸检查
    if width > 1900 or height > 1100:
        return False
    
    # 置信度二次过滤
    if confidence < 0.45:
        return False
    
    return True


def run_yolo_inference(frame, session, input_name, input_size=416):
    """使用 ONNX Runtime 运行 YOLOv12 推理"""
    # 预处理
    img = cv2.resize(frame, (input_size, input_size))
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img = img.transpose(2, 0, 1).astype(np.float32) / 255.0
    img = np.expand_dims(img, axis=0)
    
    # 推理
    outputs = session.run(None, {input_name: img})
    predictions = outputs[0][0].T
    
    # 解析输出
    boxes = predictions[:, :4]
    scores = predictions[:, 4:]
    
    class_ids = np.argmax(scores, axis=1)
    confidences = np.max(scores, axis=1)
    
    # 过滤低置信度
    mask = confidences > YOLO_CONF_THRESHOLD
    boxes = boxes[mask]
    confidences = confidences[mask]
    class_ids = class_ids[mask]
    
    # 只保留人员类别 (class_id == 0)
    person_mask = class_ids == 0
    boxes = boxes[person_mask]
    confidences = confidences[person_mask]
    
    # 转换坐标格式
    x_center, y_center, width, height = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
    x1 = x_center - width / 2
    y1 = y_center - height / 2
    x2 = x_center + width / 2
    y2 = y_center + height / 2
    
    # NMS
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
            
            detections = np.column_stack([x1, y1, x2, y2, confidences])
            return detections
    
    return np.array([])


def get_depth_at_point(depth_map, x, y, window_size=DEPTH_WINDOW_SIZE):
    """获取指定点的深度值 (中值滤波)"""
    if depth_map is None:
        return None
    
    h, w = depth_map.shape[:2]
    
    half_win = window_size // 2
    y_min = max(0, y - half_win)
    y_max = min(h, y + half_win + 1)
    x_min = max(0, x - half_win)
    x_max = min(w, x + half_win + 1)
    
    region = depth_map[y_min:y_max, x_min:x_max]
    valid_depths = region[region > 0]
    
    if len(valid_depths) == 0:
        return None
    
    depth_value = np.median(valid_depths)
    
    if depth_value < DEPTH_MIN_VALID or depth_value > DEPTH_MAX_VALID:
        return None
    
    return float(depth_value)


def select_primary_target(detections, frame_width):
    """选择主要追踪目标 (最大面积 + 最近中心)"""
    if len(detections) == 0:
        return None
    
    best_score = -1
    best_idx = 0
    frame_center_x = frame_width / 2
    
    for i, det in enumerate(detections):
        x1, y1, x2, y2, conf = det
        
        # 计算面积得分
        area = (x2 - x1) * (y2 - y1)
        
        # 计算中心偏移得分 (越靠近中心越好)
        center_x = (x1 + x2) / 2
        center_offset = abs(center_x - frame_center_x) / frame_center_x
        center_score = 1 - center_offset
        
        # 综合得分 = 面积 * 中心得分 * 置信度
        score = area * center_score * conf
        
        if score > best_score:
            best_score = score
            best_idx = i
    
    return detections[best_idx]


# ============== 主函数 ==============
def main():
    """主函数：视觉目标追踪"""
    cap = None
    try:
        print("\n" + "="*60)
        print("🎯 视觉目标追踪系统 (YOLOv12 + P100R)")
        print("   目标位置将发送给 Nav2 进行追踪导航")
        print("="*60)
        
        # 初始化相机
        print("\n初始化 Berxel 相机...")
        cap = initialize_camera()
        print("✅ 相机初始化成功")
        
        print("\n系统就绪！")
        print("\n控制键:")
        print("  - 'q': 退出程序")
        print("  - 's': 保存截图")
        print("  - 'p': 暂停/继续")
        print("="*60 + "\n")
        
        # 状态变量
        frame_count = 0
        last_detections = []
        fps_start_time = time.time()
        fps_frame_count = 0
        current_fps = 0
        paused = False
        screenshot_count = 0
        
        # 追踪目标
        primary_target = None
        target_distance = None
        
        while True:
            # 暂停处理
            if paused:
                key = cv2.waitKey(100) & 0xFF
                if key == ord('p'):
                    paused = False
                    print("\n▶️  继续运行")
                elif key == ord('q'):
                    break
                continue
            
            loop_start = time.time()
            
            # 获取图像
            frame = cap.get_frame()
            depth = cap.get_depth()
            
            if frame is None:
                print("无法获取图像帧")
                continue
            
            h, w = frame.shape[:2]
            display = frame.copy()
            
            frame_count += 1
            fps_frame_count += 1
            
            # 缩放深度图
            depth_resized = None
            if depth is not None and depth.size > 0:
                depth_resized = cv2.resize(depth, (w, h), interpolation=cv2.INTER_NEAREST)
            
            # ========== YOLO 检测 ==========
            try:
                if frame_count % (SKIP_FRAMES + 1) == 0:
                    detections = run_yolo_inference(frame, session, input_name, YOLO_INPUT_SIZE)
                    
                    scale_x = w / YOLO_INPUT_SIZE
                    scale_y = h / YOLO_INPUT_SIZE
                    
                    last_detections = []
                    for det in detections:
                        x1 = int(det[0] * scale_x)
                        y1 = int(det[1] * scale_y)
                        x2 = int(det[2] * scale_x)
                        y2 = int(det[3] * scale_y)
                        confidence = float(det[4])
                        
                        if is_valid_person((x1, y1, x2, y2), confidence, h, w):
                            last_detections.append((x1, y1, x2, y2, confidence))
                    
                    # 选择主要追踪目标
                    if last_detections:
                        primary_target = select_primary_target(
                            np.array(last_detections), w
                        )
                    else:
                        primary_target = None
                        target_distance = None
                
                # 绘制检测结果
                for i, detection in enumerate(last_detections):
                    x1, y1, x2, y2, confidence = detection
                    center_x = int((x1 + x2) / 2)
                    center_y = int((y1 + y2) / 2)
                    
                    # 获取深度
                    depth_value = get_depth_at_point(depth_resized, center_x, center_y)
                    distance_m = depth_value / DEPTH_SCALE if depth_value else None
                    
                    # 判断是否为主目标
                    is_primary = (primary_target is not None and 
                                  np.allclose([x1, y1, x2, y2, confidence], primary_target, atol=1))
                    
                    if is_primary:
                        target_distance = distance_m
                        color = (0, 0, 255)  # 红色 - 主目标
                        thickness = 3
                        label = f"TARGET {confidence:.2f}"
                    else:
                        color = (0, 255, 0)  # 绿色 - 其他人员
                        thickness = 2
                        label = f"Person {confidence:.2f}"
                    
                    if distance_m is not None:
                        label += f" {distance_m:.2f}m"
                    
                    # 绘制
                    cv2.rectangle(display, (x1, y1), (x2, y2), color, thickness)
                    cv2.putText(display, label, (x1, y1 - 10),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                    cv2.drawMarker(display, (center_x, center_y),
                                 color, cv2.MARKER_CROSS, 15 if is_primary else 10, 2)
                
                # 计算 FPS
                if fps_frame_count >= 30:
                    elapsed = time.time() - fps_start_time
                    current_fps = fps_frame_count / elapsed
                    fps_start_time = time.time()
                    fps_frame_count = 0
                
                # ========== 显示信息 ==========
                # FPS
                cv2.putText(display, f"FPS: {current_fps:.1f}", (10, 30),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
                
                # 检测数量
                cv2.putText(display, f"People: {len(last_detections)}", (10, 60),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                
                # 目标信息
                if primary_target is not None:
                    target_info = "TARGET: "
                    if target_distance is not None:
                        target_info += f"{target_distance:.2f}m"
                    else:
                        target_info += "detecting..."
                    cv2.putText(display, target_info, (10, 90),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                    
                    # 目标位置 (归一化坐标，供Nav2使用)
                    tx1, ty1, tx2, ty2, _ = primary_target
                    target_center_x = (tx1 + tx2) / 2 / w  # 0-1
                    target_center_y = (ty1 + ty2) / 2 / h  # 0-1
                    cv2.putText(display, f"Pos: ({target_center_x:.2f}, {target_center_y:.2f})", 
                              (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                else:
                    cv2.putText(display, "No target", (10, 90),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.6, (128, 128, 128), 2)
                
                # 底部标题
                cv2.putText(display, "Visual Tracker - YOLOv12", (w//2-120, h-20),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                
                # 缩放显示
                display_h = int(h * DISPLAY_SCALE)
                display_w = int(w * DISPLAY_SCALE)
                display_resized = cv2.resize(display, (display_w, display_h))
                
                cv2.imshow('Visual Tracker', display_resized)
                
                # 键盘控制
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('s'):
                    screenshot_count += 1
                    filename = f'tracker_screenshot_{screenshot_count}.png'
                    cv2.imwrite(filename, display)
                    print(f"\n📸 截图保存: {filename}")
                elif key == ord('p'):
                    paused = True
                    print("\n⏸️  已暂停（按'p'继续）")
                
                # 帧率控制
                elapsed = time.time() - loop_start
                target_delay = 1.0 / TARGET_FPS
                if elapsed < target_delay:
                    time.sleep(target_delay - elapsed)
                    
            except Exception as e:
                print(f"主循环错误: {e}")
                continue
                
    except Exception as e:
        print(f"系统错误: {e}")
        import traceback
        traceback.print_exc()
        
    finally:
        print("\n" + "="*60)
        print("🧹 清理资源...")
        
        cv2.destroyAllWindows()
        if cap:
            cap.release()
        
        print(f"\n📊 运行统计:")
        print(f"  - 总帧数: {frame_count}")
        print(f"  - 平均FPS: {current_fps:.1f}")
        
        print("\n✅ 系统已关闭")
        print("="*60)


if __name__ == "__main__":
    main()
