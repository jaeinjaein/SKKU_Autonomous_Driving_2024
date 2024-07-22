import cv2
import math
import json
import numpy as np
#from parking_code.LiDAR_util import *
from math import sin, cos, pi, pow, sqrt, atan
import random


current_scan = None     # 스캔중인 프레임의 실시간 점들
current_result = []


window_width = 360
window_height = 360
max_distance = 2500
min_distance = 50
min_angle = 0
max_angle = 180
scale = window_width / (2 * max_distance)  # 360 / (2 * 2500) = 720 / 10000 = 0.072


def get_distance(pt_a, pt_b):  # 원 좌표계로 표현된 두 점 사이의 거리를 반환하는 함수(검증 O)
    return sqrt((pt_a[1] - pt_b[1])**2 + (pt_a[2] - pt_b[2])**2)


def recursive_find(to_remove_list, term):
    global current_scan, current_result
    for to_remove_idx in to_remove_list:
        new_remove_list = []
        current_result.append(to_remove_idx)
        for find_idx in [item for item in range(0, len(current_scan)) if not (item in current_result)]:
            distance = get_distance(pt_a=current_scan[to_remove_idx], pt_b=current_scan[find_idx])
            if distance < term:
                new_remove_list.append(find_idx)
        recursive_find(new_remove_list, term)


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


def getDistanceRange(scan, minDist, maxDist):
    data = np.array(scan)
    condition = np.where((data[:, 2] < maxDist) & (data[:, 2] > minDist))
    return data[condition]

def getAngleDistanceRange(scan, minAngle, maxAngle, minDist, maxDist):
    data = np.array(scan)
    condition = np.where((data[:, 1] < maxAngle) & (data[:, 1] > minAngle) & (data[:, 2] < maxDist) & (data[:, 2] > minDist))
    return data[condition]

def getAngleRange(scan, minAngle, maxAngle):
    data = np.array(scan)
    condition = np.where((data[:, 1] < maxAngle) & (data[:, 1] > minAngle))
    return data[condition]

def polar_to_cartesian(points):
    """
    원 좌표계 (polar coordinates) 배열을 직교좌표계 (cartesian coordinates)로 변환
    :param points: np.array of shape (n, 3) with columns [index, angle, distance]
    :return: np.array of shape (n, 3) with columns [index, x, y]
    """
    # 각도와 거리를 추출
    indices = points[:, 0]
    angles = points[:, 1]
    distances = points[:, 2]

    # 각도를 라디안으로 변환
    angles_rad = np.radians(angles)

    # 직교좌표 계산
    x = distances * np.cos(angles_rad)
    y = distances * np.sin(angles_rad)

    # 결과 배열 생성
    cartesian_points = np.column_stack((indices, x, y))
    return cartesian_points

def calculate_x_difference(points):
    """
    x, y 좌표 중 x축의 최대값과 최소값을 가지는 점의 x축 차이를 계산
    :param points: np.array of shape (n, 2) with columns [x, y]
    :return: x축 최대값과 최소값의 차이
    """
    # x 좌표를 추출
    x_coords = points[:, 1]

    # x 좌표의 최대값과 최소값 계산
    max_x = np.max(x_coords)
    min_x = np.min(x_coords)

    # 최대값과 최소값의 차이 계산
    x_difference = max_x - min_x

    return x_difference


def is_car_y(points):
    y_coords = points[:, 2]
    min_y_index = np.argmin(y_coords)
    min_y_point = points[min_y_index]
    y_distance = int(min_y_point[2])
    return y_distance, ((abs(int(min_y_point[1])) < 50) and (y_distance > 500))


def lidar_analyze(scan, img=None):
    global current_scan, current_result
    if img is not None:
        cv2.circle(img, (int(window_width / 2), int(window_height / 2)), int(window_height / 2),
                   (255, 255, 255), 1)
    # print(scan)
    scan_cut = getDistanceRange(scan, min_distance, max_distance)
    scan_cut = getAngleRange(scan_cut, min_angle, max_angle)
    cartesian_points = polar_to_cartesian(scan_cut)
    # print(cartesian_points)
    ptr_set_list = []
    current_scan = cartesian_points
    while len(current_scan) > 0:
        recursive_find(to_remove_list=[0], term=120)
        ptr_set_list.append(current_scan[np.array(current_result)])
        current_scan = np.delete(current_scan, np.array(current_result), axis=0)
        current_result = []
    for ptr_set in ptr_set_list:
        if len(ptr_set) < 5:
            continue
        # color = random_colors[random.randint(0, 5)]
        x_diff = calculate_x_difference(ptr_set)
        distance, car_y_tf = is_car_y(ptr_set)
        if x_diff > 200 and car_y_tf:
            for ptr in ptr_set:
                x, y = int(ptr[1]), int(ptr[2])
                x = int(window_width / 2 + x * scale)
                y = int(window_height / 2 - y * scale)
                cv2.circle(img, (x, y), 2, (255,255,255), -1)
            print(distance, len(ptr_set))
            return distance, ptr_set
    return -1, None


if __name__ == '__main__':
    random_colors = [(255, 255, 0), (255, 0, 255), (0, 255, 255), (255, 0, 0), (0, 255, 0), (0, 0, 255)]
    with open('./lidar_data.json', 'r') as f:
        lidar_datas= json.load(f)
        for scan in lidar_datas:
            img = np.zeros((window_height, window_width, 3), np.uint8)
            lidar_analyze(scan, img=img)

        # print(len(ptr_set_list))
        # print(len(ptr_set_list))
            cv2.imshow('', img)
            cv2.waitKey(100)