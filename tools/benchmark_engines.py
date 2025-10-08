#!/usr/bin/env python3
"""Lightweight benchmark for PyTorch, ONNXRuntime (CPU) and ONNXRuntime (OpenVINO if available).

Usage:
    python tools/benchmark_engines.py --duration 20

This script uses a synthetic input (RGB image) to avoid camera dependencies and runs inference loops
for the requested duration, printing average FPS per engine.
"""
import time
import argparse
import numpy as np


def bench_pytorch(duration, img):
    from ultralytics import YOLO
    model = YOLO('yolov8n.pt')
    # warmup
    model(img)
    t0 = time.time()
    n = 0
    while time.time() - t0 < duration:
        _ = model(img, verbose=False)
        n += 1
    elapsed = time.time() - t0
    print(f"PyTorch: {n} runs in {elapsed:.2f}s -> {n/elapsed:.2f} FPS")


def bench_onnxruntime(session, duration, input_name, img_np):
    # warmup
    session.run(None, {input_name: img_np})
    t0 = time.time()
    n = 0
    while time.time() - t0 < duration:
        session.run(None, {input_name: img_np})
        n += 1
    elapsed = time.time() - t0
    print(f"ONNXRuntime ({session.get_providers()[0]}): {n} runs in {elapsed:.2f}s -> {n/elapsed:.2f} FPS")


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--duration', type=int, default=10, help='Duration per engine in seconds')
    parser.add_argument('--img-size', type=int, default=640, help='Square input image size')
    args = parser.parse_args()

    # synthetic input:
    H = W = args.img_size
    # For PyTorch/ultralytics, pass HWC uint8 BGR image
    img_hwc = np.zeros((H, W, 3), dtype=np.uint8)
    # For ONNXRuntime, NCHW float32
    img_np = np.zeros((1, 3, H, W), dtype=np.float32)

    # PyTorch benchmark
    try:
        print('\n=== PyTorch benchmark ===')
        bench_pytorch(args.duration, img_hwc)
    except Exception as e:
        print('PyTorch benchmark failed:', e)

    # ONNXRuntime benchmark
    try:
        import onnxruntime as ort
        print('\n=== ONNXRuntime benchmarks ===')
        onnx_path = 'yolov8n.onnx'
        sess_options = ort.SessionOptions()
        # try OpenVINO provider first
        providers = []
        try:
            sess_ov = ort.InferenceSession(onnx_path, sess_options, providers=['OpenVINOExecutionProvider'])
            input_name = sess_ov.get_inputs()[0].name
            bench_onnxruntime(sess_ov, args.duration, input_name, img_np)
        except Exception as e:
            print('OpenVINO provider not available or failed:', e)
            try:
                sess_cpu = ort.InferenceSession(onnx_path, sess_options, providers=['CPUExecutionProvider'])
                input_name = sess_cpu.get_inputs()[0].name
                bench_onnxruntime(sess_cpu, args.duration, input_name, img_np)
            except Exception as e2:
                print('ONNX CPU provider failed:', e2)
    except Exception as e:
        print('onnxruntime not available:', e)
