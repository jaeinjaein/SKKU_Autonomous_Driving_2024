import cv2
from time import time_ns
import numpy as np
import math
from numpy.polynomial.polynomial import Polynomial
import matplotlib.pyplot as plt

from util import tools

MAKE_LOG = False
LOG_FOLDER = './log/meanline'
HEIGHT_CROP = (267, 420)


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
    

def inf_angle(model, image, running_average):
    p1, p2 = None, None
    point_left, point_mid, point_right = None, None, None
    line1_ang, line2_ang = -9999.0,-9999.0
    line1_pts, line2_pts = np.array([]), np.array([])
    results = model(image, conf=0.4, half=True)
    image = results[0].plot()

    h, w, c = image.shape
    for idx, box in enumerate(results[0].boxes):
        if box.cls == 4:
            return
    for idx, box in enumerate(results[0].boxes):
        if box.cls == 2:
            p1 = find_sideline(image, results[0].masks[idx].xy[0], w * 0.5, h * 0.75, w, h * 0.95)
            if p1 != None:
                x_1 = np.linspace(h//2, h, 50)
                y_1 = p1(x_1)
                #for x_p, y_p in zip(x_1, y_1):
                #    cv2.circle(image, (int(y_p), int(x_p)), 5, (0,255,0), -1)

                line1_pts = tools.convert_bev_points(image, np.array(list(zip(y_1,x_1))).reshape(-1,1,2))

                point_right = (int(p1(h-1)), h-1)
                #cv2.circle(image, point_right, 10, (255,255,255), -1)

            p2 = find_sideline(image, results[0].masks[idx].xy[0], 0, h * 0.75, w * 0.5, h * 0.95)
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
    global MAKE_LOG, HEIGHT_CROP, before_ang
    print(image.shape)
    segment_points = sorted(segment_points, key=lambda x : x[1])
    segment_points.reverse()
    x = list()
    y = list()
    if MAKE_LOG:
    	t = time_ns()
    	np.save(f'{LOG_FOLDER}/{t}', segment_points)
    	cv2.imwrite(f'{LOG_FOLDER}/{t}.jpg', image)
    for idx, point in enumerate(segment_points[:-1]):
        cnt = 1
        x_o, y_o = point
        while True:
            if len(segment_points) == idx+cnt:
                break
            x_n, y_n = segment_points[idx+cnt]
            if abs(y_n - y_o) < 10 and abs(x_n - x_o) > 100:
                cv2.circle(image, (int((x_o + x_n) // 2), int((y_o + y_n) // 2)), radius=2, color=(0, 0, 255), thickness=-1)
                x.append(int((x_o + x_n) // 2))
                y.append(int((y_o + y_n) // 2))
                break
            cnt += 1
            if cnt == 5:
                break
                
    x, y = np.array(y),np.array(x)
    degree = 2
    mask = (x >= HEIGHT_CROP[0]) & (x <= HEIGHT_CROP[1])
    x = x[mask]
    y = y[mask]
    p = Polynomial.fit(x, y, degree)
    y_ans_1 = p(367)
    y_ans_2 = p(420)
    result_degree = math.degrees(math.atan((y_ans_1-y_ans_2)/(420-367)))
    cv2.line(image, (int(y_ans_1),int(367)), (int(y_ans_2),int(420)), color=(255, 0, 255), thickness=10, lineType=None, shift=None)
    return result_degree

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
