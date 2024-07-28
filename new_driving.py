from ultralytics import YOLO
import cv2
import numpy as np
from inference import inf_angle_mainline
import time

if __name__ == '__main__':
    test_video = './records/2024-07-27-203224/2024-07-27-203224_main.mp4'
    cap = cv2.VideoCapture(test_video)
    model = YOLO('./models/yolov8m-ep200-unf-d4.pt', task='segment')
    model.to('mps')
    model.half()
    while True:
        ret, frame = cap.read()
        if ret:
            image, drawed_img, line1_ang, line2_ang, mid_bias = inf_angle_mainline(model=model, image=frame,
                                                                                   SAMPLING_RATE=0.9,
                                                                                   bev_width_offset=0.19,
                                                                                   bev_height_offset=0.1,
                                                                                   line_name='rrline',
                                                                                   degree=2, visualize=True, conf=0.2)
            result_img = np.concatenate([image, drawed_img], axis=1)
            cv2.imshow('frame', result_img)
            if cv2.waitKey(30) & 0xFF == ord('s'):
                time.sleep(5)
