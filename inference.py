import cv2
from time import time_ns
import numpy as np
import math
from numpy.polynomial.polynomial import Polynomial
import matplotlib.pyplot as plt
import time
from util import tools

MAKE_LOG = False
LOG_FOLDER = './log/meanline'
HEIGHT_CROP = (540, 1000)

INFERENCE_COLOR = [(0,0,255), (255,0,0), (0,255,0), (255,255,0), (0,255,255)]

#SAMPLING_RATE = 0.8

class RunningAverage:
    def __init__(self):
        self.total = 0
        self.count = 0

    def add_value(self, value):
        self.total += value
        self.count += 1
        return self.total / self.count

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


def inf_angle_mainline(model, image, SAMPLING_RATE, bev_width_offset, bev_height_offset, degree):
    global INFERENCE_COLOR
    line1_ang, line2_ang = None, None
    t0 = time.time_ns()
    results = model(image, conf=0.2, half=True)
    drawed_img = results[0].plot()
    h, w, c = image.shape
    drawed_img = tools.convert_bev(image, bev_width_offset, bev_height_offset)
    point_r, point_l = None, None
    line1_pts, line2_pts = [], []
    t1 = time.time_ns()
    for idx, box in enumerate(results[0].boxes):
        if box.cls > 2:
            continue
        seg_points = results[0].masks[idx].xy[0]
        #for seg_point in seg_points:
        #    cv2.circle(image, (int(seg_point[0]), int(seg_point[1])), 5, INFERENCE_COLOR[int(box.cls)], -1)
        if box.cls == 2:
            line1_pts, line2_pts = calculate_mainline((h, w), seg_points, SAMPLING_RATE)
    t2 = time.time_ns()
    print(f"t2 - t1 : {(t2 - t1) / 1000000}ms")
    if len(line1_pts) >= 2:
        line1_pts = np.array(line1_pts, dtype=np.float32)
        line1_pts = tools.convert_bev_points((h, w), line1_pts.reshape(-1, 1, 2), bev_width_offset, bev_height_offset)
        line1_pts = line1_pts.reshape(-1,2)
        for line1_pt in line1_pts:
            cv2.circle(drawed_img, (int(line1_pt[0]), int(line1_pt[1])), 3, (0,0,127), -1)
        line1_pts_x, line1_pts_y = line1_pts[:,0], line1_pts[:,1]
        p_r = Polynomial.fit(line1_pts_y, line1_pts_x, degree)
        y_r = np.linspace(h//2, h, 50)
        x_r = p_r(y_r)
        for i, _ in enumerate(x_r):
            cv2.circle(drawed_img, (int(x_r[i]), int(y_r[i])), 5, (0, 0, 255), -1)
        point_r1 = [p_r(h * SAMPLING_RATE), h * SAMPLING_RATE]
        point_r2 = [p_r(h * SAMPLING_RATE + 0.01), h * SAMPLING_RATE + 0.01]
        line1_ang = math.degrees(math.atan((point_r2[0] - point_r1[0]) / (point_r1[1] - point_r2[1])))
        point_r = p_r(360)
    if len(line2_pts) >= 2:
        line2_pts = np.array(line2_pts, dtype=np.float32)
        line2_pts = tools.convert_bev_points((h, w), line2_pts.reshape(-1, 1, 2), bev_width_offset, bev_height_offset)
        line2_pts = line2_pts.reshape(-1,2)
        for line2_pt in line2_pts:
            cv2.circle(drawed_img, (int(line2_pt[0]), int(line2_pt[1])), 3, (0,127,0), -1)
        line2_pts_x, line2_pts_y = line2_pts[:,0], line2_pts[:,1]
        p_l = Polynomial.fit(line2_pts_y, line2_pts_x, degree)
        y_l = np.linspace(h//2, h, 50)
        x_l = p_l(y_l)
        for i, _ in enumerate(x_l):
            cv2.circle(drawed_img, (int(x_l[i]), int(y_l[i])), 5, (0,255,0), -1)
        point_l1 = [p_l(h * SAMPLING_RATE), h * SAMPLING_RATE]
        point_l2 = [p_l(h * SAMPLING_RATE + 0.01), h * SAMPLING_RATE + 0.01]
        line2_ang = math.degrees(math.atan((point_l2[0] - point_l1[0]) / (point_l1[1] - point_l2[1])))
        point_l = p_l(360)
    if point_r != None and point_l != None and abs(line1_ang) < 5 and abs(line2_ang) < 5:
        mid_point_x = (point_r + point_l) // 2
        mid_bias = (mid_point_x - 320) // 30
        return image, drawed_img, line1_ang, line2_ang, mid_bias
    else:
        return image, drawed_img, line1_ang, line2_ang, None
    #image = cv2.resize(image, (orig_w, orig_h))
    
    t3 = time.time_ns()
    print(f"t3 - t2 : {(t3 - t2) / 1000000}ms")
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
    if len(line_r) < 2 and len(line_l) < 2:
        line_r = find_sideline(segment_points, w * 0.5, h * (SAMPLING_RATE - 0.3), w, h * SAMPLING_RATE)
        line_l = find_sideline(segment_points, 0, h * (SAMPLING_RATE - 0.3), w * 0.5, h * SAMPLING_RATE)
    return line_r, line_l
                
    #x, y = np.array(y),np.array(x)
    #degree = 2
    #mask = (x >= HEIGHT_CROP[0]) & (x <= HEIGHT_CROP[1])
    #x = x[mask]
    #y = y[mask]
    #p = Polynomial.fit(x, y, degree)
    #y_ans_1 = p(367)
    #y_ans_2 = p(420)
    #result_degree = math.degrees(math.atan((y_ans_1-y_ans_2)/(420-367)))
    #cv2.line(image, (int(y_ans_1),int(367)), (int(y_ans_2),int(420)), color=(255, 0, 255), thickness=10, lineType=None, shift=None)
    #return result_degree

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
