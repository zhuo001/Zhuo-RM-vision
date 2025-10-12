#!/usr/bin/env python3
"""
person_detect.py 参数快速切换工具
可以在不同预设配置之间快速切换
"""

import re
import sys

PRESETS = {
    "high-performance": {
        "name": "高性能模式（高FPS，较少检测）",
        "params": {
            "SKIP_FRAMES": 4,
            "YOLO_CONF_THRESHOLD": 0.55,
            "YOLO_MAX_DETECTIONS": 50,
            "MIN_ASPECT_RATIO": 0.5,
            "MAX_ASPECT_RATIO": 5.0,
            "area_min": 0.005,
            "area_max": 0.95,
            "width_min": 60,
            "height_min": 100,
            "conf_check": 0.55,
        },
        "expected_fps": "50-80",
    },
    "balanced": {
        "name": "平衡模式（中等FPS和检测）",
        "params": {
            "SKIP_FRAMES": 2,
            "YOLO_CONF_THRESHOLD": 0.45,
            "YOLO_MAX_DETECTIONS": 75,
            "MIN_ASPECT_RATIO": 0.4,
            "MAX_ASPECT_RATIO": 6.0,
            "area_min": 0.003,
            "area_max": 0.97,
            "width_min": 50,
            "height_min": 80,
            "conf_check": 0.50,
        },
        "expected_fps": "40-60",
    },
    "high-quality": {
        "name": "高质量模式（更多检测，较低FPS）",
        "params": {
            "SKIP_FRAMES": 1,
            "YOLO_CONF_THRESHOLD": 0.40,
            "YOLO_MAX_DETECTIONS": 100,
            "MIN_ASPECT_RATIO": 0.3,
            "MAX_ASPECT_RATIO": 8.0,
            "area_min": 0.002,
            "area_max": 0.98,
            "width_min": 40,
            "height_min": 60,
            "conf_check": 0.45,
        },
        "expected_fps": "35-50",
    },
    "ultra-sensitive": {
        "name": "超敏感模式（检测所有目标，最低FPS）",
        "params": {
            "SKIP_FRAMES": 0,
            "YOLO_CONF_THRESHOLD": 0.30,
            "YOLO_MAX_DETECTIONS": 150,
            "MIN_ASPECT_RATIO": 0.2,
            "MAX_ASPECT_RATIO": 10.0,
            "area_min": 0.001,
            "area_max": 0.99,
            "width_min": 30,
            "height_min": 40,
            "conf_check": 0.35,
        },
        "expected_fps": "20-35",
    },
}

def show_presets():
    print("=" * 60)
    print("可用的预设配置：")
    print("=" * 60)
    for key, preset in PRESETS.items():
        print(f"\n{key}:")
        print(f"  名称: {preset['name']}")
        print(f"  预期 FPS: {preset['expected_fps']}")
        print(f"  跳帧: {preset['params']['SKIP_FRAMES']}")
        print(f"  置信度: {preset['params']['YOLO_CONF_THRESHOLD']}")
        print(f"  最大检测: {preset['params']['YOLO_MAX_DETECTIONS']}")
    print("\n" + "=" * 60)

def apply_preset(preset_name):
    if preset_name not in PRESETS:
        print(f"❌ 错误：未找到预设 '{preset_name}'")
        show_presets()
        return False
    
    preset = PRESETS[preset_name]
    params = preset['params']
    
    print(f"\n应用预设: {preset['name']}")
    print(f"预期 FPS: {preset['expected_fps']}")
    print("\n参数:")
    for key, value in params.items():
        print(f"  {key}: {value}")
    
    # 读取文件
    with open('person_detect.py', 'r', encoding='utf-8') as f:
        content = f.read()
    
    # 替换参数（简化版，实际使用需要更精确的替换）
    print("\n⚠️  注意：此工具仅显示配置，需要手动修改 person_detect.py")
    print("\n建议手动修改以下参数：")
    print(f"SKIP_FRAMES = {params['SKIP_FRAMES']}")
    print(f"YOLO_CONF_THRESHOLD = {params['YOLO_CONF_THRESHOLD']}")
    print(f"YOLO_MAX_DETECTIONS = {params['YOLO_MAX_DETECTIONS']}")
    print(f"MIN_ASPECT_RATIO = {params['MIN_ASPECT_RATIO']}")
    print(f"MAX_ASPECT_RATIO = {params['MAX_ASPECT_RATIO']}")
    
    return True

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("用法: python3 switch_preset.py <preset_name>")
        show_presets()
        print("\n示例:")
        print("  python3 switch_preset.py high-performance")
        print("  python3 switch_preset.py high-quality")
    else:
        apply_preset(sys.argv[1])
