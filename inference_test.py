from ultralytics import YOLO
import cv2
import numpy as np
import time




model = YOLO('./models/yolov8m-sub-ep200-unf-d2.pt', task='detect')
model.to('mps')
video = cv2.VideoCapture('./records/2024-07-27-215131/2024-07-27-215131_sub.mp4')

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