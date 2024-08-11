from ultralytics import YOLO
import cv2
import numpy as np
import time
from threading import Thread
from multiprocessing import Process
import matplotlib.pyplot as plt
import torch






def init_model(model):
    for _ in range(10):
        model(np.zeros((360, 640, 3), np.int8), device='mps', conf=0.2)

def preprocess_frame(frame):
    frame = cv2.resize(frame, (640, 640))
    frame = frame[:, :, ::-1].transpose(2, 0, 1)  # BGR to RGB, HWC to CHW
    frame = np.ascontiguousarray(frame)
    frame = torch.tensor(frame, dtype=torch.float16).to('mps')
    frame = frame.unsqueeze(0)  # Add batch dimension
    return frame / 255.0


def cal_100_inf(model, video, dummy=False):
    inftime_list = []
    i = 0
    while True:
        ret, frame = video.read()
        if ret:
            t1 = time.time_ns()
            results = model(frame, device='mps', conf=0.25)
            t2 = time.time_ns()
            if not dummy:
                inftime_list.append((t2 - t1) / 1e+6)
            i += 1
        else:
            break
    if not dummy:
        plt.plot(inftime_list)
        plt.show()


if __name__ == '__main__':
    model_1 = YOLO('./models/yolov8m-ep200-unf-d4.pt', task='segment')
    model_2 = YOLO('./models/yolov10x.pt', task='detect')
    init_model(model_1)
    init_model(model_2)
    model_1.to('mps')
    model_2.to('mps')
    test_cap = cv2.VideoCapture('./records/2024-08-03-125616/2024-08-03-125616_main.mp4')
    test_sub_cap = cv2.VideoCapture('./records/2024-08-03-125616/2024-08-03-125616_sub.mp4')
    p1 = Thread(target=cal_100_inf, args=(model_1, test_cap, False))
    p2 = Thread(target=cal_100_inf, args=(model_2, test_sub_cap, True))
    p1.start()
    p2.start()
    p1.join()
    p2.join()



# while True:
#     ret, frame = video.read()
#     if ret:
#         results = model(frame, device='mps', conf=0.2)
#         frame_infer = results[0].plot()
#         for idx, box in enumerate(results[0].boxes):
#             if box.cls == 2:
#                 print('find walk')
#                 print('')
#             if box.cls == 1:
#                 print('find sign')
#                 sign_width = int(box.xyxy[0][2] - box.xyxy[0][0])
#                 sign_height = int(box.xyxy[0][3] - box.xyxy[0][1])
#                 red_box = frame[
#                             int(box.xyxy[0][1] + (sign_height * 1 / 3)):int(box.xyxy[0][3] - (sign_height * 1 / 3)),
#                             int(box.xyxy[0][0]):int(box.xyxy[0][2] - (sign_width * 2 / 3))]
#
#                 yellow_box = frame[
#                             int(box.xyxy[0][1] + (sign_height * 1 / 3)):int(box.xyxy[0][3] - (sign_height * 1 / 3)),
#                             int(box.xyxy[0][0] + (sign_width * 1 / 3)):int(box.xyxy[0][2] - (sign_width * 1 / 3))]
#                 green_box = frame[
#                             int(box.xyxy[0][1] + (sign_height * 1 / 3)):int(box.xyxy[0][3] - (sign_height * 1 / 3)),
#                             int(box.xyxy[0][0] + (sign_width * 2 / 3)):int(box.xyxy[0][2])]
#                 red_hsv = cv2.cvtColor(red_box, cv2.COLOR_BGR2HSV)
#                 yellow_hsv = cv2.cvtColor(yellow_box, cv2.COLOR_BGR2HSV)
#                 green_hsv = cv2.cvtColor(green_box, cv2.COLOR_BGR2HSV)
#                 print(f'red_hsv_value : {np.average(red_hsv[:,:,2])}')
#                 print(f'yellow_h_avg : {np.average(yellow_hsv[:,:,2])}')
#                 print(f'green_h_avg : {np.average(green_hsv[:,:,2])}')
#                 cv2.rectangle(frame_infer, (int(box.xyxy[0][0]), int(box.xyxy[0][1])),
#                               (int(box.xyxy[0][2]), int(box.xyxy[0][3])), (255,255,255), 5)
#                 result_image = np.concatenate([red_box, yellow_box, green_box], axis=1)
#
#                 cv2.imshow('frame', result_image)
#                 if cv2.waitKey(30) & 0xFF == ord('q'):
#                     time.sleep(5)

test_image = cv2.imread('./hsv_test.png')


def sign_brightness(area):
    h, w, c = area.shape
    red_box = test_image[
        int(h * (1 / 3)):int(h * (2 / 3)),
        0:int(w * 1/3)]

    yellow_box = test_image[
        int(h * (1 / 3)):int(h * (2 / 3)),
        int(w * 1/3):int(w * 2/3)]
    green_box = test_image[
        int(h * (1 / 3)):int(h * (2 / 3)),
        int(w * 2/3):w]
    red_hsv = cv2.cvtColor(red_box, cv2.COLOR_BGR2HSV)
    yellow_hsv = cv2.cvtColor(yellow_box, cv2.COLOR_BGR2HSV)
    green_hsv = cv2.cvtColor(green_box, cv2.COLOR_BGR2HSV)
    red_v = np.average(red_hsv[:,:,2])
    yellow_v = np.average(yellow_hsv[:,:,2])
    green_v = np.average(green_hsv[:,:,2])
    print(f'red_hsv_value : {red_v}')
    print(f'yellow_h_avg : {yellow_v}')
    print(f'green_h_avg : {green_v}')
    return red_v, yellow_v, green_v