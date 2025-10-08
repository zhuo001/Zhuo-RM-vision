from ultralytics import YOLO

# 加载YOLOv8n模型
model = YOLO('yolov8n.pt')

# 打印模型信息
print("Model information:")
print(f"Model type: {model.type}")
print(f"Task: {model.task}")
print(f"Names: {model.names}")

# 运行简单的预测来验证模型
results = model(['https://ultralytics.com/images/bus.jpg'])
print("\nPrediction completed successfully!")