#!/usr/bin/env python3
"""
测试 ONNX Runtime YOLOv8 推理性能
不需要相机，使用随机图像测试
"""

import cv2
import numpy as np
import time
import onnxruntime as ort

print("=" * 60)
print("ONNX Runtime YOLOv8 推理性能测试")
print("=" * 60)

# 检查可用的 providers
available_providers = ort.get_available_providers()
print(f"\n可用的 ONNX Runtime providers: {available_providers}")

if 'ROCMExecutionProvider' in available_providers:
    providers = ['ROCMExecutionProvider', 'CPUExecutionProvider']
    print("✅ 使用 ROCMExecutionProvider (AMD GPU 加速)")
else:
    providers = ['CPUExecutionProvider']
    print("ℹ️ 使用 CPUExecutionProvider (CPU 优化)")

# 加载 ONNX 模型
onnx_model_path = 'yolov8n.onnx'
print(f"\n加载模型: {onnx_model_path}")
session = ort.InferenceSession(onnx_model_path, providers=providers)

# 获取输入输出信息
input_name = session.get_inputs()[0].name
input_shape = session.get_inputs()[0].shape
output_names = [o.name for o in session.get_outputs()]
print(f"输入: {input_name}, 形状: {input_shape}")
print(f"输出: {output_names}")

# 配置
YOLO_INPUT_SIZE = 416
NUM_WARMUP = 5
NUM_INFERENCE = 50

print(f"\n推理配置:")
print(f"  输入尺寸: {YOLO_INPUT_SIZE}x{YOLO_INPUT_SIZE}")
print(f"  预热次数: {NUM_WARMUP}")
print(f"  测试次数: {NUM_INFERENCE}")

# 创建随机测试图像
test_frame = np.random.randint(0, 255, (1080, 1920, 3), dtype=np.uint8)

def run_onnx_inference_simple(frame, session, input_name, input_size=416):
    """简化的 ONNX 推理函数"""
    # 预处理
    img = cv2.resize(frame, (input_size, input_size))
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img = img.transpose(2, 0, 1).astype(np.float32) / 255.0
    img = np.expand_dims(img, axis=0)
    
    # 推理
    outputs = session.run(None, {input_name: img})
    return outputs

# 预热
print("\n🔥 预热模型...")
for i in range(NUM_WARMUP):
    _ = run_onnx_inference_simple(test_frame, session, input_name, YOLO_INPUT_SIZE)
    print(f"  预热 {i+1}/{NUM_WARMUP}", end='\r')
print("\n✅ 预热完成")

# 性能测试
print(f"\n🚀 开始性能测试 ({NUM_INFERENCE} 次推理)...")
inference_times = []

for i in range(NUM_INFERENCE):
    start_time = time.time()
    _ = run_onnx_inference_simple(test_frame, session, input_name, YOLO_INPUT_SIZE)
    inference_time = (time.time() - start_time) * 1000  # 转换为毫秒
    inference_times.append(inference_time)
    
    if (i + 1) % 10 == 0:
        avg_time = np.mean(inference_times[-10:])
        print(f"  进度: {i+1}/{NUM_INFERENCE}, 最近10次平均: {avg_time:.2f}ms ({1000/avg_time:.1f} FPS)")

# 统计结果
inference_times = np.array(inference_times)
print("\n" + "=" * 60)
print("📊 性能统计")
print("=" * 60)
print(f"平均推理时间: {np.mean(inference_times):.2f} ms")
print(f"中位数推理时间: {np.median(inference_times):.2f} ms")
print(f"最小推理时间: {np.min(inference_times):.2f} ms")
print(f"最大推理时间: {np.max(inference_times):.2f} ms")
print(f"标准差: {np.std(inference_times):.2f} ms")
print(f"\n🎯 理论最大 FPS (仅推理): {1000/np.mean(inference_times):.1f}")
print(f"🎯 实际预期 FPS (含预处理/显示): {1000/(np.mean(inference_times) + 10):.1f}")
print(f"🎯 跳帧优化后 FPS (SKIP_FRAMES=4): {1000/(np.mean(inference_times)/5 + 10):.1f}")

print("\n" + "=" * 60)
print("测试完成！")
print("=" * 60)
