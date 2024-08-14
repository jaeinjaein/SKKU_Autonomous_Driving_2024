from ultralytics import YOLO
import cv2

if __name__ == '__main__':
    model = YOLO('../models/yolov8x-sub-ep200-frz-d2.pt', task='detect')
    model.to('mps')
    cap = cv2.VideoCapture('../records/2024-08-13-145257/2024-08-13-145257_sub.mp4')
    while True:
        ret, frame = cap.read()
        if ret:
            results = model(frame, conf=0.4)
            cv2.imshow('detected', results[0].plot())
            cv2.waitKey(30)
        else:
            break