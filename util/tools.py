import serial.tools.list_ports
import os
import cv2

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
