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
        [width * 0.3, height * 0.5], 
        [width * 0.7, height * 0.5], 
        [width * 0, height * 1.0], 
        [width * 1, height * 1.0]
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
        [width * 0.3, height * 0.5], 
        [width * 0.7, height * 0.5], 
        [width * 0, height * 1.0], 
        [width * 1, height * 1.0]
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
