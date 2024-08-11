from ultralytics import YOLO
import cv2
import numpy as np
from inference import find_bev_width_offset

model = YOLO('./models/yolov8m-ep200-unf-d3.pt', task='segment')
model.to('mps')
cap = cv2.VideoCapture(1)
while True:
    ret, frame = cap.read()
    if ret:
        print(find_bev_width_offset(model, frame, 0.9, 0.1, 0.8))
