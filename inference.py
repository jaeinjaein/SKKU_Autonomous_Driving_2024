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

SAMPLING_RATE = 0.8

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


def find_sideline(image, xy_list, box_crop_min_x, box_crop_min_y, box_crop_max_x, box_crop_max_y):
    # 직선을 찾음
    x = list()
    y = list()
    result = None
    for ptr in xy_list:
        if (box_crop_min_x <= ptr[0] <= box_crop_max_x) and (box_crop_min_y <= ptr[1] <= box_crop_max_y):
            x.append(int(ptr[0]))
            y.append(int(ptr[1]))
    x, y = np.array(y),np.array(x)
    if len(x) != 0:
        try:
            degree = 1
            p = Polynomial.fit(x, y, degree)
            for x, y in zip(y,x):
                cv2.circle(image, (int(x), int(y)), 10, (255,255,255), -1)
            if True:
                result = p
        except:
            print("frm error")
    return result


def inf_angle_mainline(model, image):
    line1_ang, line2_ang = None, None
    results = model(image, conf=0.2, half=True)
    image = results[0].plot()
    
    h, w, c = image.shape
    line1_pts, line2_pts = [], []
    for idx, box in enumerate(results[0].boxes):
        if box.cls == 2:
            t1 = time.time_ns()
            line1_pts, line2_pts = calculate_mainline(image, results[0].masks[idx].xy[0])
            t2 = time.time_ns()
            print(f"{t2 - t1}ns")
    image = tools.convert_bev(image)
    if len(line1_pts) >= 2:
        line1_pts = np.array(line1_pts, dtype=np.float32)
        line1_pts = tools.convert_bev_points(image, line1_pts.reshape(-1, 1, 2))
        line1_pts = line1_pts.reshape(-1,2)
        line1_pts_x, line1_pts_y = line1_pts[:,0], line1_pts[:,1]
        degree = 2
        p_r = Polynomial.fit(line1_pts_y, line1_pts_x, degree)
        y_r = np.linspace(h//2, h, 50)
        x_r = p_r(y_r)
        for i, _ in enumerate(x_r):
            cv2.circle(image, (int(x_r[i]), int(y_r[i])), 5, (255, 0, 0), -1)
        point_r1 = [p_r(h * SAMPLING_RATE), h * SAMPLING_RATE]
        point_r2 = [p_r(h * SAMPLING_RATE + 0.01), h * SAMPLING_RATE + 0.01]
        line1_ang = math.degrees(math.atan((point_r2[0] - point_r1[0]) / (point_r1[1] - point_r2[1])))
        
    if len(line2_pts) >= 2:
        line2_pts = np.array(line2_pts, dtype=np.float32)
        line2_pts = tools.convert_bev_points(image, line2_pts.reshape(-1, 1, 2))
        line2_pts = line2_pts.reshape(-1,2)
        line2_pts_x, line2_pts_y = line2_pts[:,0], line2_pts[:,1]
        degree = 2
        p_l = Polynomial.fit(line2_pts_y, line2_pts_x, degree)
        y_l = np.linspace(h//2, h, 50)
        x_l = p_l(y_l)
        for i, _ in enumerate(x_l):
            cv2.circle(image, (int(x_l[i]), int(y_l[i])), 5, (0,255,0), -1)
        point_l1 = [p_l(h * SAMPLING_RATE), h * SAMPLING_RATE]
        point_l2 = [p_l(h * SAMPLING_RATE + 0.01), h * SAMPLING_RATE + 0.01]
        line2_ang = math.degrees(math.atan((point_l2[0] - point_l1[0]) / (point_l1[1] - point_l2[1])))
    return image, line1_ang, line2_ang

def inf_angle(model, image, running_average):
    p1, p2 = None, None
    point_left, point_mid, point_right = None, None, None
    line1_ang, line2_ang = -9999.0,-9999.0
    line1_pts, line2_pts = np.array([]), np.array([])
    results = model(image, conf=0.2, half=True)
    image = results[0].plot()

    h, w, c = image.shape
    #for idx, box in enumerate(results[0].boxes):
        #if box.cls == 4:
        #    return
    for idx, box in enumerate(results[0].boxes):
        if box.cls == 2:
            calculate_mainline(image, results[0].masks[idx].xy[0])
            p1 = find_sideline(image, results[0].masks[idx].xy[0], w * 0.5, h * 0.75, w, h * 0.9)
            if p1 != None:
                x_1 = np.linspace(h//2, h, 50)
                y_1 = p1(x_1)
                #for x_p, y_p in zip(x_1, y_1):
                #    cv2.circle(image, (int(y_p), int(x_p)), 5, (0,255,0), -1)

                line1_pts = tools.convert_bev_points(image, np.array(list(zip(y_1,x_1))).reshape(-1,1,2))

                point_right = (int(p1(h-1)), h-1)
                #cv2.circle(image, point_right, 10, (255,255,255), -1)

            p2 = find_sideline(image, results[0].masks[idx].xy[0], 0, h * 0.75, w * 0.5, h * 0.9)
            if p2 != None:
                x_2 = np.linspace(h//2, h, 50)
                y_2 = p2(x_2)
                #for x_p, y_p in zip(x_2, y_2):
                #    cv2.circle(image, (int(y_p), int(x_p)), 5, (255,0,0), -1)
                line2_pts = tools.convert_bev_points(image, np.array(list(zip(y_2,x_2))).reshape(-1,1,2))
                point_left = (int(p2(h-1)), h-1)
                cv2.circle(image, point_left, 10, (255,255,255), -1)
            if point_right != None and point_left != None:
                avg = running_average.add_value(point_right[0] - point_left[0])
            if point_right != None and running_average.count != 0:
                point_mid = (int(point_right[0] - (running_average.total / running_average.count)//2), h-1)
                cv2.circle(image, point_mid, 10, (0,0,255), -1)
    image = tools.convert_bev(image)
    for pt in line1_pts:
        cv2.circle(image, (int(pt[0][0]), int(pt[0][1])), 5, (255,0,0), -1)
    for pt in line2_pts:
        cv2.circle(image, (int(pt[0][0]), int(pt[0][1])), 5, (255,0,0), -1)
    if len(line1_pts) >= 2:
        line1_ang = math.degrees(math.atan((line1_pts[1][0][0] - line1_pts[0][0][0]) / (line1_pts[0][0][1] - line1_pts[1][0][1])))
    if len(line2_pts) >= 2:
        line2_ang = math.degrees(math.atan((line2_pts[1][0][0] - line2_pts[0][0][0]) / (line2_pts[0][0][1] - line2_pts[1][0][1])))

    #print(line1_ang, line2_ang)
    #cv2.imshow('', image)
    #cv2.waitKey(10)
    return image, point_mid, line1_ang, line2_ang




def calculate_mainline(image, segment_points):
    h, w, c = image.shape
    segment_points = sorted(segment_points, key=lambda x : x[1], reverse=True)
    x = list()
    y = list()
    point_cnt = 1
    line_r, line_l = [], []
    for idx, point in enumerate(segment_points[:-1]):
        cnt = 1
        x_o, y_o = point
        if y_o >= h * 0.9:
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
        if y_o <= h * 0.5:
            break
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
