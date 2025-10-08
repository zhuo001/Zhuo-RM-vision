#!/usr/bin/env python3
"""Simple OpenVINO benchmark for an ONNX model.
Loads an ONNX model with OpenVINO Python API and runs synthetic-inference for a duration.
Outputs average FPS.
"""
import time
import numpy as np
import os

MODEL_PATH = os.environ.get('OV_MODEL', 'yolov8n.onnx')
DEVICE = os.environ.get('OV_DEVICE', 'GPU')  # try GPU (iGPU via OpenCL). Fallback to CPU if needed.
DURATION = 10.0

def main():
    try:
        import openvino as ov
    except Exception as e:
        print('OpenVINO not available:', e)
        return 1

    core = ov.Core()
    print('OpenVINO version:', ov.__version__)
    print('Available devices:', core.available_devices)

    if not os.path.exists(MODEL_PATH):
        print('Model not found:', MODEL_PATH)
        return 2

    # Read model and compile
    model = core.read_model(model=MODEL_PATH)
    try:
        compiled = core.compile_model(model, device_name=DEVICE)
    except Exception as e:
        print(f'Failed to compile on device {DEVICE}:', e)
        # fallback to CPU
        print('Falling back to CPU')
        compiled = core.compile_model(model, device_name='CPU')

    # Prepare input
    input_port = compiled.inputs[0]
    shape = list(input_port.shape)
    # Replace dynamic dims with concrete values if present
    shape = [s if isinstance(s, int) else 1 for s in shape]
    print('Input shape:', shape)

    # synthetic input (float32)
    x = np.random.randn(*shape).astype(np.float32)

    # Warmup
    for _ in range(5):
        compiled([x])

    # Benchmark
    start = time.time()
    runs = 0
    while time.time() - start < DURATION:
        compiled([x])
        runs += 1

    elapsed = time.time() - start
    fps = runs / elapsed if elapsed > 0 else 0.0
    print(f'Ran {runs} in {elapsed:.2f}s -> {fps:.2f} FPS (device={compiled.device_name})')
    return 0

if __name__ == '__main__':
    raise SystemExit(main())
