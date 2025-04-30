from ultralytics import YOLO

# Load a YOLOv8n PyTorch model
model = YOLO("ball_goalpost.pt")

# Export the model
model.export(format="openvino", dynamic=True)  # creates 'yolov8n_openvino_model/'
