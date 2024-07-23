import json
import math
import cv2
import numpy as np


window_width = 360
window_height = 360
max_distance = 2500
min_distance = 50
min_angle = 0
max_angle = 180
scale = window_width / (2 * max_distance)  # 360 / (2 * 2500) = 720 / 10000 = 0.072
def polar_to_cartesian(angle, distance):
    """폴라 좌표를 카르테시안 좌표로 변환"""
    rads = math.radians(angle)
    x = distance * math.cos(rads)
    y = distance * math.sin(rads)
    return x, y

def draw_lidar_scan(image, scan):
    """라이다 스캔 데이터를 화면에 그리기"""
    results = []
    for (_, angle, distance) in scan:
        if distance > max_distance:
            continue
        x, y = polar_to_cartesian(angle, distance)
        results.append([x, y])
        x = int(window_width / 2 + x * scale)
        y = int(window_height / 2 - y * scale)
        cv2.circle(image, (x, y), 2, (0, 255, 0), -1)
    return results

with open('./lidar_data.json', 'r') as f:
    lidar_datas = json.load(f)
    for scan in lidar_datas:
        image = np.zeros((window_height, window_width, 3), np.uint8)
        draw_lidar_scan(image, scan)
        cv2.imshow('lidar', image)
        cv2.waitKey(100)