import cv2

from ultralytics import YOLO
import os
from collections import defaultdict

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from ultralytics.engine.results import Results
import torch
from typing import List, Union
import numpy as np


package_name = Path(__file__).parent.parent.stem
package_share_dir = get_package_share_directory(package_name)


device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
print(f"Using device: {device}")


# Load the YOLOv8 model
model = YOLO(os.path.join(package_share_dir,"weights/yolov8s.pt")).to(device)
cap = cv2.VideoCapture(0)

# Store the track history
track_history = defaultdict(lambda: [])


# Loop through the video frames
while cap.isOpened():
    # Read a frame from the video
    success, frame = cap.read()
    
    if not success:
        break
    
    # Run YOLOv8 tracking on the frame, persisting tracks between frames
    results:List[Results] = model.track(source=frame, persist=True, show=False)
    
    # Visualize the results on the frame
    annotated_frame = results[0].plot()
    
    
    cv2.imshow("YOLOv8 Tracking", annotated_frame)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break
   
# Release the video capture object and close the display window
cap.release()
cv2.destroyAllWindows()