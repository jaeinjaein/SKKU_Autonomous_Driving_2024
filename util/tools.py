import serial.tools.list_ports
import os
import cv2
import numpy as np
import matplotlib.pyplot as plt

device_vp = [('Lidar', '1A86:7523'), ('Arduino', '2A03:0042')]
#angle_min, angle_max = -80, 80
#steering_values = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 16, 17, 17, 17]
#bev_height_offset = 0.20 # bigger --> higher
#bev_width_offset = 0.27  # bigger --> narrow
save_statistics = True


def cam_capture():
    capture = cv2.VideoCapture(0)
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, 160)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 160)

    while cv2.waitKey(33) < 0:
        ret, frame = capture.read()
        cv2.imshow("VideoFrame", frame)

    capture.relase()
    cv2.destroyAllWindows()

def serial_ports():
    result = []
    for port in serial.tools.list_ports.comports():
        for device_info in device_vp:
            if port.hwid.find(device_info[1]) >= 0:
                result.append(f'{device_info[0]}:{port.name}')
    return result

def lidar_datas():
    return os.listdir('../lidar_dataset')

def drive_datas():
    return os.listdir('../cam_dataset')
    
def convert_bev(image, bev_width_offset, bev_height_offset):
    # 원본 이미지의 크기를 가져옵니다.
    height, width = image.shape[:2]

    # 변환 전 후의 4개의 지점을 정의합니다.
    # 변환 전 지점들 (예시 좌표입니다. 실제 도로 사진에 맞게 조정해야 합니다)
    src_points = np.float32([
        [int(width * bev_width_offset), height * (0.5 - bev_height_offset)], 
        [int(width * (1.0 - bev_width_offset)), height * (0.5 - bev_height_offset)], 
        [width * 0, height * (1.0 - bev_height_offset)], 
        [width * 1, height * (1.0 - bev_height_offset)]
    ])

    # 변환 후 지점들
    dst_points = np.float32([
        [0, 0], 
        [width, 0], 
        [0, height], 
        [width, height]
    ])

    # 변환 매트릭스를 계산합니다.
    matrix = cv2.getPerspectiveTransform(src_points, dst_points)

    # 퍼스펙티브 변환을 적용합니다.
    bird_eye_view = cv2.warpPerspective(image, matrix, (width, height))
    return bird_eye_view

def convert_bev_points(image, points, bev_width_offset, bev_height_offset):
    height, width = image.shape[:2]
    # 변환 전 후의 4개의 지점을 정의합니다.
    # 변환 전 지점들 (예시 좌표입니다. 실제 도로 사진에 맞게 조정해야 합니다)
    src_points = np.float32([
        [int(width * bev_width_offset), height * (0.5 - bev_height_offset)], 
        [int(width * (1.0 - bev_width_offset)), height * (0.5 - bev_height_offset)], 
        [width * 0, height * (1.0 - bev_height_offset)], 
        [width * 1, height * (1.0 - bev_height_offset)]
    ])

    # 변환 후 지점들
    dst_points = np.float32([
        [0, 0], 
        [width, 0], 
        [0, height], 
        [width, height]
    ])
    
    matrix = cv2.getPerspectiveTransform(src_points, dst_points)
    transformed_point = cv2.perspectiveTransform(points, matrix)

    return transformed_point

def map_to_n_levels(value):
    if value < -75:
        value = -75
    elif value > 75:
        value = 75
    
    mapped_value = int((value+75)/ 150 * 20)+1
    return mapped_value 
    
def put_message(image, msg_cnt=1, msg=['test'], show=False):
    top_left_x = 0
    top_left_y = 0
    width = 500
    height = 20 + (msg_cnt * 50)
    color = (255, 255, 255)  # white
    cv2.rectangle(image, (top_left_x, top_left_y), (top_left_x + width, top_left_y + height), color, -1)
    
    org = (top_left_x + 10, top_left_y + 50)
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 1
    font_color = (0, 0, 0)  # blck
    thickness = 2
    
    for i in range(msg_cnt):
        org = (top_left_x + 10, top_left_y + (50*(i+1)))
        cv2.putText(image, msg[i], org, font, font_scale, font_color, thickness, cv2.LINE_AA)

    if show:
        cv2.imshow("Image with Text", image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        cv2.imwrite('output_image.jpg', image)
    return image
    
def map_to_steering(value, angle_min, angle_max, steering_values):
    if value < angle_min:
        value = angle_min
    elif value > angle_max:
        value = angle_max
    step = (angle_max - angle_min) / 21
    index = int((value - angle_min) // step)
    #if index == 20 and value == max_val:
    #    index = 19
    
    return index, steering_values[index]
