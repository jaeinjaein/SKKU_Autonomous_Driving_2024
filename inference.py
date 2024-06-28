import cv2

def draw_mainline(image, segment_points):
    segment_points = sorted(segment_points, key=lambda x : x[1])
    segment_points.reverse()
    for idx, point in enumerate(segment_points[:-1]):
        cnt = 1
        x, y = point
        while True:
            if len(segment_points) == idx+cnt:
                break
            x_n, y_n = segment_points[idx+cnt]
            if abs(y_n - y) < 3 and abs(x_n - x) > 100:
                cv2.circle(image, (int((x + x_n) // 2), int((y + y_n) // 2)), radius=5, color=(0, 0, 255), thickness=-1)
                break
            cnt += 1
            if cnt == 5:
                break

def inference_image(model, img):
    results = model(img, conf=0.2)
    img_inf = results[0].plot()
    rrline_idx = -1
    for idx, box in enumerate(results[0].boxes):
        if int(box.cls) == 2:
            rrline_idx = idx
    if rrline_idx != -1:
        draw_mainline(img_inf, results[0].masks[rrline_idx].xy[0])
    return img_inf