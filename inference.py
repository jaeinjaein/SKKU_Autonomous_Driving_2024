import cv2
from time import time_ns
import numpy as np
import math
from numpy.polynomial.polynomial import Polynomial

MAKE_LOG = False
LOG_FOLDER = './log/meanline'
HEIGHT_CROP = (400, 630)

def convert_ats(angle):
    # convert angle (-360, 360, float) -> steering value (0, 15, int)
    steering = -1
    
    return steering


def calculate_mainline(image, segment_points):
    global MAKE_LOG, HEIGHT_CROP, before_ang
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
                cv2.circle(image, (int((x_o + x_n) // 2), int((y_o + y_n) // 2)), radius=5, color=(0, 0, 255), thickness=-1)
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
    y_ans_1 = p(550)
    y_ans_2 = p(630)
    result_degree = math.degrees(math.atan((y_ans_1-y_ans_2)/(630-550)))
    cv2.line(image, (int(y_ans_1),int(550)), (int(y_ans_2),int(630)), color=(255, 0, 255), thickness=10, lineType=None, shift=None)
    return result_degree

def inference_image(model, img):
    results = model(img, conf=0.6)
    img_inf = results[0].plot()
    rrline_idx = -1
    for idx, box in enumerate(results[0].boxes):
        if int(box.cls) == 2:
            rrline_idx = idx
    if rrline_idx != -1:
        result_degree = calculate_mainline(img_inf, results[0].masks[rrline_idx].xy[0])
    return img_inf, result_degree
