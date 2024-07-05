import serial.tools.list_ports
import os
import cv2
import numpy as np
import matplotlib.pyplot as plt

device_vp = [('Lidar', '1A86:7523'), ('Arduino', '2A03:0042')]

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
    
def convert_bev(image):
    # 원본 이미지의 크기를 가져옵니다.
    height, width = image.shape[:2]

    # 변환 전 후의 4개의 지점을 정의합니다.
    # 변환 전 지점들 (예시 좌표입니다. 실제 도로 사진에 맞게 조정해야 합니다)
    src_points = np.float32([
        [int(width * 0.27), height * 0.25], 
        [int(width * 0.73), height * 0.25], 
        [width * 0, height * 0.75], 
        [width * 1, height * 0.75]
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

def convert_bev_points(image, points):
    
    height, width = image.shape[:2]
    # 변환 전 후의 4개의 지점을 정의합니다.
    # 변환 전 지점들 (예시 좌표입니다. 실제 도로 사진에 맞게 조정해야 합니다)
    src_points = np.float32([
        [int(width * 0.27), height * 0.25], 
        [int(width * 0.73), height * 0.25], 
        [width * 0, height * 0.75], 
        [width * 1, height * 0.75]
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
