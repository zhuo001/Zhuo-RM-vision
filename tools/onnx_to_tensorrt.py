#!/usr/bin/env python3
"""
ONNX模型转换为TensorRT引擎

⚠️ 注意: 此脚本仅适用于NVIDIA GPU环境
⚠️ AMD 780M用户请使用ONNXRuntime + OpenVINO方案

TensorRT是NVIDIA专用的深度学习推理优化引擎，可以显著提升推理速度。
但它只能在NVIDIA GPU上运行，不支持AMD GPU。

使用方法（需要NVIDIA GPU + CUDA + TensorRT）:
    python tools/onnx_to_tensorrt.py --onnx models/yolov8n.onnx --output models/yolov8n.engine --fp16

依赖安装（NVIDIA环境）:
    # 方法1: 使用官方TensorRT Python包
    pip install tensorrt
    
    # 方法2: 使用NVIDIA Docker容器
    docker pull nvcr.io/nvidia/tensorrt:23.12-py3
"""

import argparse
import sys
from pathlib import Path

print("=" * 80)
print("⚠️  TensorRT转换工具 - 仅适用于NVIDIA GPU")
print("=" * 80)
print()

# 检查TensorRT是否可用
try:
    import tensorrt as trt
    import pycuda.driver as cuda
    import pycuda.autoinit
    TRT_AVAILABLE = True
    print(f"✓ TensorRT版本: {trt.__version__}")
except ImportError as e:
    TRT_AVAILABLE = False
    print("✗ TensorRT未安装或不可用")
    print()
    print("如果您使用的是NVIDIA GPU，请安装TensorRT:")
    print("  方法1: pip install tensorrt")
    print("  方法2: 使用NVIDIA TensorRT Docker容器")
    print()
    print("如果您使用的是AMD GPU (如AMD 780M)，请使用以下方案:")
    print("  1. 导出ONNX: python tools/export_to_onnx.py --model yolov8n.pt --fp16")
    print("  2. 使用ONNXRuntime: python person_detect.py --engine onnx-openvino")
    print()
    sys.exit(1)

import numpy as np


class TensorRTEngineBuilder:
    """TensorRT引擎构建器"""
    
    def __init__(self, onnx_path: str, engine_path: str, fp16: bool = False, int8: bool = False, 
                 max_batch_size: int = 1, workspace_size: int = 4):
        """
        Args:
            onnx_path: ONNX模型路径
            engine_path: 输出TensorRT引擎路径
            fp16: 是否使用FP16精度
            int8: 是否使用INT8精度（需要校准数据）
            max_batch_size: 最大batch大小
            workspace_size: 工作空间大小（GB）
        """
        self.onnx_path = onnx_path
        self.engine_path = engine_path
        self.fp16 = fp16
        self.int8 = int8
        self.max_batch_size = max_batch_size
        self.workspace_size = workspace_size * (1 << 30)  # 转换为字节
        
        self.logger = trt.Logger(trt.Logger.INFO)
    
    def build_engine(self):
        """构建TensorRT引擎"""
        print(f"[1/4] 创建TensorRT Builder...")
        builder = trt.Builder(self.logger)
        network = builder.create_network(1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH))
        parser = trt.OnnxParser(network, self.logger)
        
        print(f"[2/4] 解析ONNX模型: {self.onnx_path}")
        with open(self.onnx_path, 'rb') as model:
            if not parser.parse(model.read()):
                print('ERROR: ONNX模型解析失败')
                for error in range(parser.num_errors):
                    print(parser.get_error(error))
                return None
        
        print(f"[3/4] 配置构建选项...")
        config = builder.create_builder_config()
        config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, self.workspace_size)
        
        # 设置精度
        if self.fp16:
            print("  - 启用FP16精度")
            config.set_flag(trt.BuilderFlag.FP16)
        
        if self.int8:
            print("  - 启用INT8精度（需要校准数据）")
            config.set_flag(trt.BuilderFlag.INT8)
            # 注意: INT8需要实现校准器，这里简化处理
            print("  ⚠ INT8校准器未实现，使用FP16替代")
            config.set_flag(trt.BuilderFlag.FP16)
        
        # 设置优化配置文件（用于动态输入）
        profile = builder.create_optimization_profile()
        
        # 假设输入名称为 "images"，形状为 [batch, 3, height, width]
        input_name = network.get_input(0).name
        input_shape = network.get_input(0).shape
        
        print(f"  - 输入名称: {input_name}")
        print(f"  - 输入形状: {input_shape}")
        
        # 设置动态输入范围
        # Min: batch=1, 640x640
        # Opt: batch=1, 640x640 (优化尺寸)
        # Max: batch=max_batch_size, 640x640
        profile.set_shape(
            input_name,
            (1, 3, 640, 640),  # min
            (1, 3, 640, 640),  # opt
            (self.max_batch_size, 3, 640, 640)  # max
        )
        config.add_optimization_profile(profile)
        
        print(f"[4/4] 构建TensorRT引擎...")
        print(f"  ⏳ 这可能需要几分钟时间...")
        
        serialized_engine = builder.build_serialized_network(network, config)
        
        if serialized_engine is None:
            print("ERROR: TensorRT引擎构建失败")
            return None
        
        # 保存引擎
        print(f"  - 保存引擎到: {self.engine_path}")
        with open(self.engine_path, 'wb') as f:
            f.write(serialized_engine)
        
        # 显示文件大小
        file_size_mb = Path(self.engine_path).stat().st_size / (1024 * 1024)
        print(f"\n✓ TensorRT引擎构建完成!")
        print(f"  - 输出路径: {self.engine_path}")
        print(f"  - 文件大小: {file_size_mb:.2f} MB")
        print(f"  - 精度: {'FP16' if self.fp16 else 'FP32'}")
        
        return serialized_engine


def main():
    parser = argparse.ArgumentParser(
        description="将ONNX模型转换为TensorRT引擎（仅NVIDIA GPU）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
⚠️ 重要提示:
  - 此工具仅适用于NVIDIA GPU环境
  - AMD 780M用户请使用: python person_detect.py --engine onnx-openvino

示例（NVIDIA GPU环境）:
  # FP16精度（推荐）
  python tools/onnx_to_tensorrt.py --onnx models/yolov8n.onnx --fp16
  
  # FP32精度（精度最高，速度较慢）
  python tools/onnx_to_tensorrt.py --onnx models/yolov8n.onnx
  
  # 指定输出路径
  python tools/onnx_to_tensorrt.py --onnx models/yolov8n.onnx --output models/yolov8n_fp16.engine --fp16

使用TensorRT引擎进行推理:
  # 注意: person_detect.py当前不支持TensorRT引擎加载
  # 需要使用专门的TensorRT推理脚本（待实现）
        """
    )
    
    parser.add_argument(
        '--onnx',
        type=str,
        required=True,
        help='输入ONNX模型路径'
    )
    
    parser.add_argument(
        '--output',
        type=str,
        default=None,
        help='输出TensorRT引擎路径 (默认: 与ONNX同名，扩展名改为.engine)'
    )
    
    parser.add_argument(
        '--fp16',
        action='store_true',
        help='使用FP16精度（推荐，2-3倍加速）'
    )
    
    parser.add_argument(
        '--int8',
        action='store_true',
        help='使用INT8精度（需要校准数据，当前未实现）'
    )
    
    parser.add_argument(
        '--max-batch-size',
        type=int,
        default=1,
        help='最大batch大小 (默认: 1)'
    )
    
    parser.add_argument(
        '--workspace-size',
        type=int,
        default=4,
        help='工作空间大小（GB）(默认: 4)'
    )
    
    args = parser.parse_args()
    
    # 检查ONNX文件
    onnx_path = Path(args.onnx)
    if not onnx_path.exists():
        print(f"错误: ONNX文件不存在: {onnx_path}")
        sys.exit(1)
    
    # 确定输出路径
    if args.output:
        engine_path = args.output
    else:
        engine_path = onnx_path.with_suffix('.engine')
    
    # 创建输出目录
    Path(engine_path).parent.mkdir(parents=True, exist_ok=True)
    
    # 构建引擎
    try:
        builder = TensorRTEngineBuilder(
            onnx_path=str(onnx_path),
            engine_path=str(engine_path),
            fp16=args.fp16,
            int8=args.int8,
            max_batch_size=args.max_batch_size,
            workspace_size=args.workspace_size
        )
        
        builder.build_engine()
        
        print("\n" + "=" * 80)
        print("提示: person_detect.py当前不支持直接加载TensorRT引擎")
        print("TensorRT引擎已保存，可用于自定义推理脚本")
        print("=" * 80)
        
    except Exception as e:
        print(f"\n错误: TensorRT引擎构建失败: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()
