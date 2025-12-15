# This script demonstrates how to train a YOLOv8n model using the Ultralytics YOLO library.
from ultralytics import YOLO

# Load a pretrained YOLO8n model
model = YOLO("yolov8n_custom.pt")  # Load the YOLOv8n model

# Perform object detection on an image
results = model("ROS2_rUBot_mecanum_ws/src/AI_Projects/my_robot_ai_identification/photos/Give/Foto_4.jpg")  # Predict on an image from test set
results[0].show()  # Display results