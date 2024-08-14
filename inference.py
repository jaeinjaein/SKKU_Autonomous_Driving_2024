import cv2
from time import time_ns
import numpy as np
import math
from numpy.polynomial.polynomial import Polynomial
import matplotlib.pyplot as plt
import time
from util import tools

from sklearn.linear_model import RANSACRegressor
from sklearn.preprocessing import PolynomialFeatures
from sklearn.pipeline import make_pipeline
from sklearn.linear_model import LinearRegression
import torch

MAKE_LOG = False
LOG_FOLDER = './log/meanline'
HEIGHT_CROP = (540, 1000)

INFERENCE_COLOR = [(0,0,255), (255,0,0), (0,255,0), (255,255,0), (0,255,255)]

def convert_ats(angle):
    # convert angle (-360, 360, float) -> steering value (0, 15, int)
    steering = -1
    
    return steering


def find_sideline(xy_list, box_crop_min_x, box_crop_min_y, box_crop_max_x, box_crop_max_y):
    result = []
    for ptr in xy_list:
        if (box_crop_min_x <= ptr[0] <= box_crop_max_x) and (box_crop_min_y <= ptr[1] <= box_crop_max_y):
            result.append(ptr)
    return result


def find_poly(h, x_1, y_1, type, degree, sampling_point_y_1, sampling_point_y_2):
    if type == 'ransac':
        polynomial_features = PolynomialFeatures(degree=degree)
        linear_regression = LinearRegression()
        ransac_regressor = RANSACRegressor(estimator=linear_regression, max_trials=100, min_samples=5,
                                           residual_threshold=5, random_state=0)
        model = make_pipeline(polynomial_features, ransac_regressor)
        r_y = np.array(y_1)
        r_y = r_y.reshape(-1, 1)
        r_x = np.array(x_1)
        r_x = r_x.reshape(-1, 1)
        model.fit(r_y, r_x)
        y_samp = np.linspace(h // 2, h, 50).reshape(-1, 1)
        x_samp = model.predict(y_samp)
        # if degree == 2:
        #     ransac_model = model.named_steps['ransacregressor']
        #     linear_model = ransac_model.estimator_
        #     coefficients = linear_model.coef_
        #     if coefficients[0][2] < 0.00000001:
        #         print('guessed linear regression coefficients too small')
        #         return find_poly(h, x_1, y_1, type, 1, sampling_point_y_1, sampling_point_y_2)
        sampling_point_x_1 = model.predict(np.array([[sampling_point_y_1]]))[0]
        sampling_point_x_2 = model.predict(np.array([[sampling_point_y_2]]))[0]
        sampling_point_m = model.predict(np.array([[h]]))[0]
        point_1 = (sampling_point_x_1, sampling_point_y_1)
        point_2 = (sampling_point_x_2, sampling_point_y_2)
        return x_samp, y_samp, point_1, point_2, sampling_point_m
    elif type == 'poly':
        p_r = Polynomial.fit(y_1, x_1, degree)
        y_samp = np.linspace(h // 2, h, 50)
        x_samp = p_r(y_samp)
        point_1 = [p_r(sampling_point_y_1), sampling_point_y_1]
        point_2 = [p_r(sampling_point_y_2), sampling_point_y_2]
        sampling_point_m = p_r(h)
        return x_samp, y_samp, point_1, point_2, sampling_point_m


def find_line(results, line_cls=1):
    line_segments = []
    for idx, box in enumerate(results[0].boxes):
        if box.cls == line_cls:
            line_segments.append(results[0].masks[idx].xy[0])
    if len(line_segments) > 0:
        line_segments.sort(key=lambda x: cv2.contourArea(x), reverse=True)
        if cv2.contourArea(line_segments[0]) > 20000:
            return line_segments[0]
        print('found, but size error')
    if len(line_segments) == 0:
        print('not found')
    return None


def find_bev_width_offset(model, image, SAMPLING_RATE, bev_height_offset, conf=0.8):
    '''
    1. Inference
    2. bev_width_offset 값 0.001씩 늘려가면서 차선 찾기 -> 외부 리스트에 양쪽차선 기울기 차이 저장
    3. 차이가 가장 적은 index 추출
    4. 차이가 가장 적은 bev값 추출
    '''
    line1_ang, line2_ang = None, None
    results = model(image, conf=conf, device='mps', verbose=False)
    h, w, c = image.shape
    point_r, point_l = None, None
    angle_diffs = []
    line1_pts, line2_pts = [], []
    line_segments = []
    for idx, box in enumerate(results[0].boxes):
        if box.cls == 1:
            line_segments.append(results[0].masks[idx].xy[0])
    if len(line_segments) > 0:
        line_segments.sort(key=lambda x: cv2.contourArea(x), reverse=True)
        if cv2.contourArea(line_segments[0]) > 20000:
            line1_pts, line2_pts = calculate_mainline((h, w), line_segments[0], SAMPLING_RATE)
        else:
            print('found, but size error')
    line1_pts = np.array(line1_pts, dtype=np.float32)
    line2_pts = np.array(line2_pts, dtype=np.float32)
    if len(line1_pts) >= 5 and len(line2_pts) >= 5:
        for bev_width_offset_1000 in range(100, 301, 1):
            bev_width_offset = bev_width_offset_1000 / 1000.0
            line1_pts_bev = tools.convert_bev_points((h, w), line1_pts.reshape(-1, 1, 2), bev_width_offset, bev_height_offset)
            line1_pts_bev = line1_pts_bev.reshape(-1,2)
            line1_pts_x, line1_pts_y = line1_pts_bev[:,0], line1_pts_bev[:,1]

            point_r1_y = h * (SAMPLING_RATE - 0.05)
            point_r2_y = h * SAMPLING_RATE

            x_r, y_r, point_r1, point_r2, point_r = find_poly(h, line1_pts_x, line1_pts_y, 'ransac', 1, point_r1_y,
                                                              point_r2_y)
            line1_ang = math.degrees(math.atan((point_r2[0] - point_r1[0]) / (point_r1[1] - point_r2[1])))

            line2_pts_bev = tools.convert_bev_points((h, w), line2_pts.reshape(-1, 1, 2), bev_width_offset, bev_height_offset)
            line2_pts_bev = line2_pts_bev.reshape(-1, 2)
            line2_pts_x, line2_pts_y = line2_pts_bev[:, 0], line2_pts_bev[:, 1]

            point_l1_y = h * (SAMPLING_RATE - 0.05)
            point_l2_y = h * SAMPLING_RATE

            x_l, y_l, point_l1, point_l2, point_l = find_poly(h, line2_pts_x, line2_pts_y, 'ransac', 1, point_l1_y,
                                                              point_l2_y)

            line2_ang = math.degrees(math.atan((point_l2[0] - point_l1[0]) / (point_l1[1] - point_l2[1])))
            angle_diffs.append(abs(line1_ang - line2_ang))
        angle_diffs = np.array(angle_diffs)
        # print(f'angle_diffs: {angle_diffs}')
        # print(f'angle_min : {angle_diffs[np.argmin(angle_diffs)]}, idx : {np.argmin(angle_diffs)}')
        # print(f'angle_max : {angle_diffs[np.argmax(angle_diffs)]}, idx : {np.argmax(angle_diffs)}')
        return np.argmin(angle_diffs)
        # cv2.imshow('', results[0].plot())
        # cv2.waitKey(30)
    else:
        print('can\'t fine two line.')
        return None

def inf_angle_mainline(model, image, SAMPLING_RATE, bev_width_offset, bev_height_offset, line_name, degree, visualize=False, conf=0.8, mid_x=325):
    global INFERENCE_COLOR
    line1_ang, line2_ang = None, None
    t0 = time.time_ns()
    results = model(image,conf=conf, device='mps', verbose=False)
    h, w, c = image.shape
    if visualize:
        drawed_img = results[0].plot()
        image = drawed_img
        drawed_img = tools.convert_bev(drawed_img, bev_width_offset, bev_height_offset)
    else:
        drawed_img = np.zeros((h, w, 3), dtype=np.uint8)
    line_cls = -1
    if line_name == 'rrline':
        line_cls = 1
    else:
        line_cls = 0
    point_r, point_l = None, None
    line1_pts, line2_pts = [], []
    t1 = time.time_ns()
    print(f"t1 - t0 : {(t1 - t0) / 1000000}ms")

    line_segments = []
    for idx, box in enumerate(results[0].boxes):
        if box.cls == line_cls:
            line_segments.append(results[0].masks[idx].xy[0])
    if len(line_segments) > 0:
        line_segments.sort(key=lambda x: cv2.contourArea(x), reverse=True)
        if cv2.contourArea(line_segments[0]) > 20000:
            line1_pts, line2_pts = calculate_mainline((h, w), line_segments[0], SAMPLING_RATE)
        else:
            print('found, but size error')
    if len(line_segments) == 0:
        print('not found')
    #
    # for idx, box in enumerate(results[0].boxes):
    #     if box.cls > 2:
    #         continue
    #     seg_points = results[0].masks[idx].xy[0]
    #     #for seg_point in seg_points:
    #     #    cv2.circle(image, (int(seg_point[0]), int(seg_point[1])), 5, INFERENCE_COLOR[int(box.cls)], -1)
    #     #print(line_cls)
    #     if box.cls == line_cls:
    #         line1_pts, line2_pts = calculate_mainline((h, w), seg_points, SAMPLING_RATE)
    #
    t2 = time.time_ns()
    print(f"t2 - t1 : {(t2 - t1) / 1000000}ms")
    if len(line1_pts) >= 5:
        line1_pts = np.array(line1_pts, dtype=np.float32)
        line1_pts = tools.convert_bev_points((h, w), line1_pts.reshape(-1, 1, 2), bev_width_offset, bev_height_offset)
        line1_pts = line1_pts.reshape(-1,2)
        for line1_pt in line1_pts:
            cv2.circle(drawed_img, (int(line1_pt[0]), int(line1_pt[1])), 3, (0,0,127), -1)
        line1_pts_x, line1_pts_y = line1_pts[:,0], line1_pts[:,1]

        point_r1_y = h * (SAMPLING_RATE - 0.05)
        point_r2_y = h * SAMPLING_RATE

        x_r, y_r, point_r1, point_r2, point_r = find_poly(h, line1_pts_x, line1_pts_y, 'ransac', degree, point_r1_y,
                                                          point_r2_y)

        for i, _ in enumerate(x_r):
            cv2.circle(drawed_img, (int(x_r[i]), int(y_r[i])), 5, (0, 0, 255), -1)

        line1_ang = math.degrees(math.atan((point_r2[0] - point_r1[0]) / (point_r1[1] - point_r2[1])))

        # print(f'line1_pts : {len(line1_pts)}')
    if len(line2_pts) >= 5:
        line2_pts = np.array(line2_pts, dtype=np.float32)
        line2_pts = tools.convert_bev_points((h, w), line2_pts.reshape(-1, 1, 2), bev_width_offset, bev_height_offset)
        line2_pts = line2_pts.reshape(-1,2)
        for line2_pt in line2_pts:
            cv2.circle(drawed_img, (int(line2_pt[0]), int(line2_pt[1])), 3, (0,127,0), -1)
        line2_pts_x, line2_pts_y = line2_pts[:,0], line2_pts[:,1]

        point_l1_y = h * (SAMPLING_RATE - 0.05)
        point_l2_y = h * SAMPLING_RATE

        x_l, y_l, point_l1, point_l2, point_l = find_poly(h, line2_pts_x, line2_pts_y, 'ransac', degree, point_l1_y,
                                                          point_l2_y)

        for i, _ in enumerate(x_l):
            cv2.circle(drawed_img, (int(x_l[i]), int(y_l[i])), 5, (0, 255, 0), -1)

        line2_ang = math.degrees(math.atan((point_l2[0] - point_l1[0]) / (point_l1[1] - point_l2[1])))

        # print(f'line2_pts : {len(line2_pts)}')

    t3 = time.time_ns()
    print(f"t3 - t2 : {(t3 - t2) / 1000000}ms")
    if point_r != None and point_l != None and abs(line1_ang) < 7 and abs(line2_ang) < 7:
        mid_point_x = (point_r + point_l) // 2
        mid_bias = (mid_point_x - mid_x) // 30
        return image, drawed_img, line1_ang, line2_ang, mid_bias
    else:
        return image, drawed_img, line1_ang, line2_ang, None
    #image = cv2.resize(image, (orig_w, orig_h))

    return image, drawed_img, line1_ang, line2_ang, None


def calculate_mainline(image_size, segment_points, SAMPLING_RATE):
    h, w = image_size
    segment_points = sorted(segment_points, key=lambda x : x[1], reverse=True)
    point_cnt = 1
    line_r, line_l = [], []
    for idx, point in enumerate(segment_points[:-1]):
        cnt = 1
        x_o, y_o = point
        if y_o >= h * SAMPLING_RATE:
            continue
        while True:
            if len(segment_points) == idx+cnt:
                break
            x_n, y_n = segment_points[idx+cnt]
            if abs(y_n - y_o) < 10 and abs(x_n - x_o) > 100:
                point_cnt += 1
                if x_n > x_o:
                    line_r.append([int(x_n), int(y_n)])
                else:
                    line_l.append([int(x_n), int(y_n)])
                break
            cnt += 1
            if cnt == 5:
                break
        if y_o <= h * (SAMPLING_RATE - 0.2):
            break
    if len(line_r) < 5 and len(line_l) < 5:
        line_r = find_sideline(segment_points, w * 0.5, h * (SAMPLING_RATE - 0.3), w, h * SAMPLING_RATE)
        line_l = find_sideline(segment_points, 0, h * (SAMPLING_RATE - 0.3), w * 0.5, h * SAMPLING_RATE)
    return line_r, line_l


def inference_image(model, img):
    results = model(img, conf=0.8)
    img_inf = results[0].plot()
    rrline_idx = -1
    for idx, box in enumerate(results[0].boxes):
        if int(box.cls) == 2:
            rrline_idx = idx
    if rrline_idx != -1:
        result_degree = calculate_mainline(img_inf, results[0].masks[rrline_idx].xy[0])
    return img_inf, result_degree
