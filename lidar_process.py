import cv2
import math
import json
import numpy as np
#from parking_code.LiDAR_util import *
from math import sin, cos, pi, pow, sqrt, atan
import random
import matplotlib.pyplot as plt


current_scan = None     # 스캔중인 프레임의 실시간 점들
current_result = []


window_width = 360
window_height = 360
max_distance = 2500
min_distance = 50
min_angle = 0
max_angle = 360
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


def get_angle(point_a, point_b):
    x_diff = float(point_b[1] - point_a[1])
    y_diff = float(point_b[2] - point_a[2])
    try:
        return atan(y_diff / x_diff) * 180 / pi
    except ZeroDivisionError:
        return None


def find_side_car(points, for_visualize=False):
    '''
    차량의 옆면을 찾고, 분산을 통해 일자인 부분을 추출하는 함수
    조건 1) 모든 y좌표가 0보다 크거나, 모든 y좌표가 0보다 작다
    조건 2) 판단된 점들의 너비가 500mm 이상이다.
    조건 3) 좌측부터 연속된 점들의 각도를 20개 이상 추출 가능하여야 한다. (구분 가능한 점이 21개 이상)
    위 3개의 조건을 만족하면, 점들을 좌측부터 갯수 기준 5개 set로 분할하고, set마다의 기울기 분산을 구한다.
    기울기 분산이 총 4개라면(=기울기 값이 20개), 가장 작은 분산을 가진 기울기의 평균을 구한다. (avg_angle)
    위 함수는 Step 3에서 뒤로 들어갈 때 두 차의 기울기 평균을 구하고, 각도에 따라서 조향각을 줄 것이다.
    0에 가까울수록, 차가 수평으로 잘 들어가고 있음을 의미한다.
    마이너스 --> 우측/좌측 조향 (실제로 구해봐야함)
    플러스 --> 좌측/우측 조향 (실제로 해봐야함)
    '''
    y_coords = points[:, 2]
    car_pos = -1
    if np.all(y_coords < 0):  #
        car_pos = 0
    elif np.all(y_coords > 0):
        car_pos = 1
    else:
        return None, None
    object_width = calculate_x_difference(points)
    if not object_width > 300:
        return None, None
    points_sorted = sorted(points, key=lambda p:p[1])
    angle_list = []
    for idx, point in enumerate(points_sorted[:-1]):
        angle = get_angle(points_sorted[idx], points_sorted[idx + 1])
        if angle is not None:
            angle_list.append(angle)
    if len(angle_list) < 20:
        return None, None
    var_list = []
    for i in range(5):
        var_list.append(np.var(angle_list[int(len(angle_list) * 0.2 * (i)):int(len(angle_list) * 0.2 * (i+1))]))
    idx = np.argmin(np.abs(np.array(var_list)))
    avg_angle = np.average(angle_list[int(len(angle_list) * 0.2 * idx):int(len(angle_list) * 0.2 * (idx + 1))])
    result_points = points_sorted[int(len(angle_list)*0.2*idx):int(len(angle_list)*0.2*(idx+1))]
    print(f'car_pos : {car_pos}, angle : {avg_angle}')
    if for_visualize:
        return car_pos, result_points
    else:
        return car_pos, avg_angle

def lidar_analyze_test(scan, img=None):
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
        # distance, car_y_tf = is_car_y(ptr_set)
        # x_diff = calculate_x_difference(ptr_set)
        car_pos, car_object = find_side_car(ptr_set, True)
        if car_object is not None:
            for ptr in car_object:
                x, y = int(ptr[1]), int(ptr[2])
                x = int(window_width / 2 + x * scale)
                y = int(window_height / 2 - y * scale)
                cv2.circle(img, (x, y), 2, (255,255,255), -1)


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
        distance, car_y_tf = is_car_y(ptr_set)
        x_diff = calculate_x_difference(ptr_set)

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
    with open('./2024-07-28-194218_lidar.json', 'r') as f:
        lidar_datas= json.load(f)
        for scan in lidar_datas:
            img = np.zeros((window_height, window_width, 3), np.uint8)
            lidar_analyze_test(scan, img=img)

        # print(len(ptr_set_list))
        # print(len(ptr_set_list))
            cv2.imshow('', img)
            cv2.waitKey(50)