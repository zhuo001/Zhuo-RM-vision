#!/usr/bin/env python3
"""
YOLOv8模型导出为ONNX格式

支持:
- 动态batch和动态输入尺寸
- FP16量化（减小模型大小，提高推理速度）
- 模型验证
- 适配ONNXRuntime和OpenVINO

使用方法:
    python tools/export_to_onnx.py --model yolov8n.pt --output models/yolov8n.onnx --fp16
"""

import argparse
from pathlib import Path
import sys

try:
    from ultralytics import YOLO
    import onnx
    import onnxruntime as ort
except ImportError as e:
    print(f"错误: 缺少必要的依赖包: {e}")
    print("请安装: pip install ultralytics onnx onnxruntime")
    sys.exit(1)


def export_yolov8_to_onnx(
    model_path: str,
    output_path: str,
    imgsz: int = 640,
    dynamic: bool = True,
    half: bool = False,
    simplify: bool = True,
    opset: int = 12
):
    """
    将YOLOv8模型导出为ONNX格式
    
    Args:
        model_path: 输入的.pt模型路径
        output_path: 输出的.onnx模型路径
        imgsz: 输入图像尺寸（默认640）
        dynamic: 是否使用动态输入尺寸
        half: 是否使用FP16量化
        simplify: 是否简化ONNX图
        opset: ONNX opset版本（默认12，兼容OpenVINO）
    """
    print(f"[1/4] 加载YOLOv8模型: {model_path}")
    model = YOLO(model_path)
    
    print(f"[2/4] 导出为ONNX格式...")
    print(f"  - 输入尺寸: {imgsz}x{imgsz}")
    print(f"  - 动态输入: {dynamic}")
    print(f"  - FP16量化: {half}")
    print(f"  - 简化图: {simplify}")
    print(f"  - ONNX opset: {opset}")
    
    # Ultralytics的export方法会自动处理
    export_path = model.export(
        format='onnx',
        imgsz=imgsz,
        dynamic=dynamic,
        half=half,
        simplify=simplify,
        opset=opset
    )
    
    print(f"[3/4] 验证ONNX模型...")
    
    # 加载并检查ONNX模型
    onnx_model = onnx.load(export_path)
    onnx.checker.check_model(onnx_model)
    print("  ✓ ONNX模型结构有效")
    
    # 打印模型信息
    print("\n模型信息:")
    print(f"  - IR版本: {onnx_model.ir_version}")
    print(f"  - Opset版本: {onnx_model.opset_import[0].version}")
    
    # 打印输入/输出信息
    print("\n输入信息:")
    for input_tensor in onnx_model.graph.input:
        shape = [dim.dim_value if dim.dim_value > 0 else 'dynamic' 
                 for dim in input_tensor.type.tensor_type.shape.dim]
        print(f"  - {input_tensor.name}: {shape}")
    
    print("\n输出信息:")
    for output_tensor in onnx_model.graph.output:
        shape = [dim.dim_value if dim.dim_value > 0 else 'dynamic' 
                 for dim in output_tensor.type.tensor_type.shape.dim]
        print(f"  - {output_tensor.name}: {shape}")
    
    # 使用ONNXRuntime测试推理
    print("\n[4/4] 使用ONNXRuntime测试推理...")
    try:
        import numpy as np
        
        # 创建ONNXRuntime会话
        providers = ['CPUExecutionProvider']
        session = ort.InferenceSession(export_path, providers=providers)
        
        # 获取输入信息
        input_name = session.get_inputs()[0].name
        input_shape = session.get_inputs()[0].shape
        
        # 创建随机输入（batch=1）
        if dynamic:
            test_shape = [1, 3, imgsz, imgsz]
        else:
            test_shape = [int(dim) if isinstance(dim, int) or dim > 0 else 1 
                         for dim in input_shape]
        
        dummy_input = np.random.randn(*test_shape).astype(np.float32)
        
        # 运行推理
        outputs = session.run(None, {input_name: dummy_input})
        print(f"  ✓ 推理成功!")
        print(f"  - 输入形状: {test_shape}")
        print(f"  - 输出形状: {[o.shape for o in outputs]}")
        
    except Exception as e:
        print(f"  ✗ ONNXRuntime测试失败: {e}")
        print("  提示: 这不影响模型导出，但可能需要检查运行时环境")
    
    # 如果指定了不同的输出路径，移动文件
    if output_path and output_path != export_path:
        import shutil
        output_path_obj = Path(output_path)
        output_path_obj.parent.mkdir(parents=True, exist_ok=True)
        shutil.move(export_path, output_path)
        final_path = output_path
    else:
        final_path = export_path
    
    # 显示文件大小
    file_size_mb = Path(final_path).stat().st_size / (1024 * 1024)
    print(f"\n✓ 导出完成!")
    print(f"  - 输出路径: {final_path}")
    print(f"  - 文件大小: {file_size_mb:.2f} MB")
    
    return final_path


def main():
    parser = argparse.ArgumentParser(
        description="将YOLOv8模型导出为ONNX格式",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  # 基础导出（动态输入，FP32）
  python tools/export_to_onnx.py --model yolov8n.pt
  
  # FP16量化（推荐用于AMD 780M）
  python tools/export_to_onnx.py --model yolov8n.pt --fp16
  
  # 指定输出路径
  python tools/export_to_onnx.py --model yolov8n.pt --output models/yolov8n_fp16.onnx --fp16
  
  # 固定输入尺寸（用于TensorRT等需要静态输入的引擎）
  python tools/export_to_onnx.py --model yolov8n.pt --no-dynamic --imgsz 640
        """
    )
    
    parser.add_argument(
        '--model',
        type=str,
        default='yolov8n.pt',
        help='YOLOv8 .pt模型路径 (默认: yolov8n.pt)'
    )
    
    parser.add_argument(
        '--output',
        type=str,
        default=None,
        help='输出ONNX模型路径 (默认: 与输入同目录，扩展名改为.onnx)'
    )
    
    parser.add_argument(
        '--imgsz',
        type=int,
        default=640,
        help='输入图像尺寸 (默认: 640)'
    )
    
    parser.add_argument(
        '--no-dynamic',
        action='store_true',
        help='禁用动态输入尺寸（某些推理引擎需要）'
    )
    
    parser.add_argument(
        '--fp16',
        action='store_true',
        help='使用FP16量化（推荐用于AMD 780M，减小模型大小，提高速度）'
    )
    
    parser.add_argument(
        '--no-simplify',
        action='store_true',
        help='禁用ONNX图简化'
    )
    
    parser.add_argument(
        '--opset',
        type=int,
        default=12,
        help='ONNX opset版本 (默认: 12，兼容OpenVINO 2022+)'
    )
    
    args = parser.parse_args()
    
    # 检查输入文件
    if not Path(args.model).exists():
        print(f"错误: 找不到模型文件: {args.model}")
        sys.exit(1)
    
    # 导出模型
    try:
        export_yolov8_to_onnx(
            model_path=args.model,
            output_path=args.output,
            imgsz=args.imgsz,
            dynamic=not args.no_dynamic,
            half=args.fp16,
            simplify=not args.no_simplify,
            opset=args.opset
        )
    except Exception as e:
        print(f"\n错误: 导出失败: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()
