import json
import matplotlib.pyplot as plt
from ultralytics import YOLO
import cv2
from inference import inf_angle_mainline
from util.tools import map_to_steering
import numpy as np



mode = 'video'
video_name = '2024-08-03-125344'
ref_steering_values=[3,4,4,5,6,7,7,8,8,9,10,11,12,12,12,13,13,14,14,15,16]
steering_values=[3,4,4,5,6,7,7,8,8,9,10,11,12,12,12,13,13,14,14,15,16]

def calculate_diff(steering_list):
    diff_list = [0]
    for i in range(1, len(steering_list)):
        diff_list.append(abs(steering_list[i] - steering_list[i - 1]))
    return diff_list


if __name__ == '__main__':
    if mode == 'json':
        with open('./records/2024-08-03-111603/2024-08-03-111603_steering.json', 'r') as f:
            result_list = json.load(f)[360:]
            diff_list = calculate_diff(result_list)
            plt.plot(result_list)
            plt.plot(diff_list)
            plt.show()
    elif mode == 'video':
        angle_list = []
        result_list = []
        model = YOLO('./models/yolov8m-ep200-unf-d4.pt', task='segment')
        model.to('mps')
        video = cv2.VideoCapture(f'./records/{video_name}/{video_name}_main.mp4')
        fault_images = []
        while True:
            ret, frame = video.read()
            if ret:
                image, drawed_img, line1_ang, line2_ang, mid_bias = inf_angle_mainline(model=model, image=frame,
                                                                                       SAMPLING_RATE=0.9,
                                                                                       bev_width_offset=0.185,
                                                                                       bev_height_offset=0.1,
                                                                                       line_name='rrline',
                                                                                       degree=1,
                                                                                       visualize=False, conf=0.5)
                steering_value = 9999.0
                first = line1_ang
                second = line2_ang
                if first is not None:
                    angle_list.append(first)

                    idx, steering_value = map_to_steering(first, angle_min=-27, angle_max=27,
                                                          steering_values=ref_steering_values)
                    if mid_bias is not None:
                        steering_value = steering_value + int(mid_bias)
                    result_list.append(steering_value)

                elif second is not None:
                    angle_list.append(second)
                    idx, steering_value = map_to_steering(second, angle_min=-27, angle_max=27,
                                                          steering_values=ref_steering_values)
                    if mid_bias is not None:
                        steering_value = steering_value + int(mid_bias)
                    if steering_value < 10:
                        steering_value = steering_value + 1
                    else:
                        steering_value = steering_value - 1
                    result_list.append(steering_value)
                else:
                    fault_images.append((len(result_list), image))
            else:
                break
        diff_list = calculate_diff(result_list)
        print(f'{len(fault_images)} images are fault')
        fig, ax1 = plt.subplots()

        ax1.plot(result_list)

        ax2 = ax1.twinx()
        ax2.plot(angle_list, 'y-')

        plt.show()
        for idx, image_info in enumerate(fault_images):
            results = model(image_info[1], conf=0.5)
            img_con = np.concatenate((image_info[1], results[0].plot()), axis=1)
            print(image_info[0])
            # for box_num, box in enumerate(results[0].boxes):
                # if box.cls == 1:
                #     print(results[0].masks)
            cv2.imshow(f'fault image {image_info[0]}', img_con)
            cv2.waitKey(-1)