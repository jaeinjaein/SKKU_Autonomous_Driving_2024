import sys
import cv2
from PyQt5.QtWidgets import QMessageBox, QApplication, QWidget, QVBoxLayout, QHBoxLayout, QGroupBox, QCheckBox, QLabel, QComboBox, QPushButton, QGridLayout
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import QTimer, Qt
import threading
import numpy as np
import serial.tools.list_ports
from inference import inf_angle_mainline
from ultralytics import YOLO
import time
import serial
from util.tools import put_message, map_to_steering
import traceback
from rplidar import RPLidar
import math
import os
import json
from lidar_process import lidar_analyze

ard_list = []
subcam_device, maincam_device, lidar_device, main_ui = None, None, None, None

class subcam():
    def __init__(self):
        self.cap = None
        self.model = YOLO('./models/yolov8m-ep100-unf-d1.pt', task='detect')
        self.model.to('mps')
        self.record = False
        self.writer_orig = None
        self.writer_inferenced = None
        self.update_img = np.zeros((360, 640, 3), dtype=np.uint8)
        self.steering_values = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20]
        self.fps = 5.0
        self.capture = False
        self.avoid_state = False
        self.traffic_state = False
        self.model(np.zeros((360, 640, 3), dtype=np.uint8), conf=0.2)
        self.found_num = 0
        
    def start_camera(self, device_index):
        if self.cap is not None:
            self.cap.release()
        self.cap = cv2.VideoCapture(device_index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)
        # print(self.cap.get(cv2.CAP_PROP_GAIN))
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        self.capture = False

    def start_record(self):
        local_time = time.localtime()
        formatted_time = time.strftime('%Y-%m-%d-%H%M%S', local_time)
        fourcc = cv2.VideoWriter_fourcc(*'H264')
        self.writer_orig = cv2.VideoWriter(f'./videos/orig/{formatted_time}_sub.mp4', fourcc, self.fps, (640, 360))
        self.writer_inferenced = cv2.VideoWriter(f'./videos/inferenced/{formatted_time}_sub.mp4', fourcc, self.fps,
                                                 (640, 360))

    def stop_record(self):
        self.writer_orig.release()
        self.writer_inferenced.release()
        self.writer_orig = None
        self.writer_inferenced = None

    def start_video(self, video_path):
        if self.cap is not None:
            self.cap.release()
        self.cap = cv2.VideoCapture(video_path)
        if self.record:
            local_time = time.localtime()
            formatted_time = time.strftime('%Y-%m-%d-%H%M%S', local_time)
            fourcc = cv2.VideoWriter_fourcc(*'H264')
            self.writer_orig = cv2.VideoWriter(f'./videos/orig/{formatted_time}.mp4', fourcc, self.fps, (640, 360))
            self.writer_inferenced = cv2.VideoWriter(f'./videos/inferenced/{formatted_time}.mp4', fourcc, self.fps, (640, 360))
        self.capture = True

    def stop_video(self):
        self.capture = False
        if self.cap is not None:
            self.cap.release()
        self.cap = None

    def stop_camera(self):
        if self.cap is not None:
            self.capture = False
            self.cap.release()
            self.cap = None
        if self.record:
            self.writer_orig.release()
            self.writer_inferenced.release()
            
    def load_params(self):
        with open('params_sub.txt', 'r') as f:
            lines = f.readlines()
            for line in lines:
                line = line.rstrip()
                name, val = line.split('=')
                exec(f'self.{line}')
        print('sub cam param updated.')
        

class maincam():
    def __init__(self):
        self.cap = None
        self.model = YOLO('./models/yolov8m-ep200-unf-d4.pt', task='segment')
        self.model.to('mps')
        self.model.half()
        self.record = False
        self.save_statistics = True
        self.SAMPLING_RATE = 0.8
        self.bev_height_offset = 0.2
        self.bev_width_offset = 0.27
        self.update_img = np.zeros((360, 1280, 3), dtype=np.uint8)
        self.angle_min = -80
        self.angle_max = 80
        self.steering_values = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20]
        self.poly_degree = 2
        self.fps = 10.0
        self.verbose = True
        self.load_params()
        self.capture = False
        self.model(np.zeros((360, 640, 3), dtype=np.uint8), conf=0.2)
        self.line_name = 'rrline'
        self.writer_orig = None
        self.writer_inferenced = None
        self.statistics_list = []

    def start_camera(self, device_index):
        if self.cap is not None:
            self.cap.release()
        self.cap = cv2.VideoCapture(device_index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        self.capture = True

    def stop_video(self):
        self.capture = False
        if self.cap is not None:
            self.cap.release()
        self.cap = None

    def stop_record(self):
        self.writer_orig.release()
        self.writer_inferenced.release()
        self.writer_orig = None
        self.writer_inferenced = None

    def start_video(self, video_path):
        if self.cap is not None:
            self.cap.release()
        self.cap = cv2.VideoCapture(video_path)
        self.capture = True
            
    def stop_camera(self):
        if self.save_statistics and len(self.statistics_list) > 0:
            local_time = time.localtime()
            formatted_time = time.strftime('%Y-%m-%d-%H%M%S', local_time)
            save_array = np.array(self.statistics_list)
            np.save(f'./log/steering_log/{formatted_time}_steering', save_array)
        if self.cap is not None:
            self.capture = False
            self.cap.release()
            self.cap = None
        if self.record:
            self.writer_orig.release()
            self.writer_inferenced.release()
            
    def load_params(self):
        with open('params.txt', 'r') as f:
            lines = f.readlines()
            for line in lines:
                line = line.rstrip()
                name, val = line.split('=')
                exec(f'self.{line}')
        print('cam param updated.')


def polar_to_cartesian(angle, distance):
    """폴라 좌표를 카르테시안 좌표로 변환"""
    rads = math.radians(angle)
    x = distance * math.cos(rads)
    y = distance * math.sin(rads)
    return x, y


def draw_lidar_scan(image, scan):
    """라이다 스캔 데이터를 화면에 그리기"""
    for (_, angle, distance) in scan:
        if distance > lidar_device.max_distance:
            continue
        x, y = polar_to_cartesian(angle, distance)
        x = int(lidar_device.window_width / 2 + x * lidar_device.scale)
        y = int(lidar_device.window_height / 2 - y * lidar_device.scale)
        cv2.circle(image, (x, y), 2, (0, 255, 0), -1)


def lidar_scan():
    while True:
        if lidar_device.lidar_session is None:
            time.sleep(0.01)
            continue
        try:
            for scan in lidar_device.lidar_session.iter_scans():
                image = np.zeros((lidar_device.window_height, lidar_device.window_width, 3), dtype=np.uint8)
                draw_lidar_scan(image, scan)
                lidar_device.update_img = image
                lidar_device.scan_data = scan
        except:
            pass
        time.sleep(0.01)


class lidar():
    def __init__(self):
        self.window_width = 360
        self.window_height = 360
        self.max_distance = 2000
        self.scale = self.window_width / (2 * self.max_distance)
        self.lidar_session = None
        self.update_img = np.zeros((self.window_width, self.window_height, 3), dtype=np.uint8)
        self.port = ''
        self.scan_data = None
        self.parking_step = 0
        self.car_a_dist = -1
        self.car_b_dist = -1
        self.car_a = None
        self.car_b = None
        self.turn_angle = 0.0

        self.BACK_PARAM = 0.015
        self.STEP3_DISTANCE_OFFSET = 1000
        self.STEP3_BACK_PARAM = 0.005



    def start_lidar(self, port):
        try:
            self.lidar_session = RPLidar(port)
            self.port = port
            info = self.lidar_session.get_info()
            print(info)
            health = self.lidar_session.get_health()
            print(health)
            self.lidar_session.start_motor()
            time.sleep(2)
        except:
            self.lidar_session = RPLidar(port)
            self.port = port
            info = self.lidar_session.get_info()
            print(info)
            health = self.lidar_session.get_health()
            print(health)
            self.lidar_session.start_motor()
            time.sleep(2)


    def stop_lidar(self):
        self.lidar_session.stop()
        self.lidar_session.stop_motor()
        self.lidar_session.disconnect()
        self.lidar_session = None


class arduino():
    def __init__(self):
        self.port = ''
        self.serial_session = None
        self.current_steering = 10
        self.before_steering = -9999
        self.current_speed = 0
        self.before_speed = -9999
        self.cps = 30.0

    def start_arduino(self, port):
        self.port = port
        self.serial_session = serial.Serial(port=self.port, baudrate=9600)

    def stop_arduino(self):
        self.serial_session.close()
        self.port = ''
        self.serial_session = None

    def send_data(self, data):
        if self.serial_session is not None:
            try:
                self.serial_session.write(data.encode())
            except serial.SerialException as e:
                print(f"Failed to send data: {e}")
                self.reconnect()
            time.sleep(1 / self.cps)

    def connect(self):
        try:
            self.serial_session = serial.Serial(port=self.port, baudrate=9600)
            print(f"Connected to {self.port} at {self.serial_session.baudrate} baud.")
        except serial.SerialException as e:
            print(f"Failed to connect to {self.port}: {e}")
            self.serial_session = None

    def disconnect(self):
        if self.serial_session and self.serial_session.is_open:
            self.serial_session.close()
            print(f"Disconnected from {self.port}")

    def reconnect(self):
        self.disconnect()
        time.sleep(0.1)  # 100ms 대기 후 재연결 시도
        self.connect()




# class arduino():
#     '''this class is original arduino structure'''
#     def __init__(self):
#         self.port = ''
#         self.serial_session = None
#
#     def start_arduino(self, port):
#         self.port = port
#         self.serial_session = serial.Serial(port=self.port, baudrate=9600)
#
#     def send_angle(self, value):
#         if self.serial_session != None:
#             try:
#                 self.serial_session.flush()
#             except:
#                 print('flush error')
#             data = 'angle%02d;' % value
#             print(self.serial_session.write(data.encode()))
#
#     def send_speed(self, value):
#         if self.serial_session != None:
#
#             try:
#                 self.serial_session.flush()
#             except:
#                 print('flush error')
#             data = 'speed'
#             if value >= 0:
#                 data += '+'
#             else:
#                 data += '-'
#             data += '%03d;' % self.abs_value(value)
#             self.serial_session.write(data.encode())
#
#     def abs_value(self, value):
#         if value >= 0:
#             return value
#         else:
#             return -1 * value
#
#     def stop_arduino(self):
#         self.serial_session.close()
#         self.port = ''
#         self.serial_session = None
#
#     def setting(self):
#         if self.serial_session != None:
#             self.serial_session.write('setting;'.encode())


# class arduino():
#     '''this class is for preparing error'''
#     def __init__(self):
#         self.port = ''
#         self.serial_session = None
#         self.lock = threading.Lock()
#
#     def start_arduino(self, port):
#         self.port = port
#         try:
#             self.serial_session = serial.Serial(port=self.port, baudrate=9600, timeout=1)
#         except Exception as e:
#             print(f"Error opening serial port: {e}")
#             traceback.print_exc()
#
#     def send_angle(self, value):
#         if self.serial_session is not None:
#             with self.lock:
#                 try:
#                     self.serial_session.flush()
#                     data = f'angle{value:02d};'
#                     self.serial_session.write(data.encode())
#                 except Exception as e:
#                     print(f"Error sending angle: {e}")
#                     self.reconnect()
#
#     def send_speed(self, value):
#         if self.serial_session is not None:
#             with self.lock:
#                 try:
#                     self.serial_session.flush()
#                     sign = '+' if value >= 0 else '-'
#                     data = f'speed{sign}{abs(value):03d};'
#                     self.serial_session.write(data.encode())
#                 except Exception as e:
#                     print(f"Error sending speed: {e}")
#                     self.reconnect()
#
#     def stop_arduino(self):
#         self.serial_session.close()
#         self.port = ''
#         self.serial_session = None
#
#     def setting(self):
#         if self.serial_session is not None:
#             with self.lock:
#                 try:
#                     self.serial_session.flush()
#                     self.serial_session.write('setting;'.encode())
#                 except Exception as e:
#                     print(f"Error sending speed: {e}")
#                     self.reconnect()
#
#     def reconnect(self):
#         print("Attempting to reconnect...")
#         try:
#             self.serial_session.close()
#         except Exception as e:
#             print(f"Error closing serial port: {e}")
#         time.sleep(0.1)
#         try:
#             self.start_arduino(self.port)
#             print("Reconnected.")
#         except Exception as e:
#             print(f"Reconnection failed: {e}")


def parking():
    lidar_device.parking_step = 1
    while lidar_device.parking_step > 0:
        if lidar_device.parking_step == 1:
            # 1단계. 직진하면서 차량 두대 발견 -> 비틀림 각도 산출
            arduino_device.current_speed = 80
            arduino_device.current_steering = 11
            if len(lidar_device.scan_data) < 10:
                time.sleep(0.1)
                continue
            distance, lidar_car = lidar_analyze(lidar_device.scan_data)  # 여기 car detect 함수 만들어서, 우측 차량과 거리 검출
            print(distance)
            if lidar_car is not None:
                if lidar_device.car_a is None:
                    lidar_device.car_a_dist, lidar_device.car_a = distance, lidar_car
                    time.sleep(2)
                elif lidar_device.car_b is None:
                    arduino_device.current_speed = 0
                    lidar_device.car_b_dist, lidar_device.car_b = distance, lidar_car
                    y_diff = lidar_device.car_b_dist - lidar_device.car_a_dist
                    lidar_device.turn_angle = math.atan(-1 * y_diff / 100) * 180 / math.pi  # 비틀림 각도 산출 (car_a의 가장 좌측, car_b의 가장 좌측 좌표를 통해)
                    print(lidar_device.turn_angle)
                    lidar_device.parking_step = 2  # 2단계(후진으로 각도 보정)으로 넘어감
                    time.sleep(2)
        elif lidar_device.parking_step == 2:
            if abs(lidar_device.turn_angle) > 30:  # 어느정도는 넘어야 보정함
                if lidar_device.turn_angle > 0:  # 차량이 우측으로 치우쳐짐
                    arduino_device.current_steering = 20
                else:  # 차량이 좌측으로 치우쳐짐
                    arduino_device.current_steering = 0
                time.sleep(0.1)
                arduino_device.current_speed = -80
                time.sleep(abs(lidar_device.turn_angle) * lidar_device.BACK_PARAM)  # 각도 보정 완료
                arduino_device.current_speed = 0
                arduino_device.current_steering = 10
            time.sleep(0.1)
            arduino_device.current_speed = -50
            time.sleep(1.5)
            arduino_device.current_speed = 0
            lidar_device.parking_step = 3
            time.sleep(1)
        elif lidar_device.parking_step == 3:
            arduino_device.current_speed = 50
            distance, lidar_car = lidar_analyze(lidar_device.scan_data)  # 다시 우측 차량 찾도록
            if lidar_car is not None:  # 우측 차량 감지 --> 차량 멈추고, 거리에 따라 다음 행동 결정
                # arduino_device.current_speed = 50
                # # if distance < 800:  # ~ 800  -> 두번에 걸쳐 수직만들기 (아직 고려 X)
                # #     pass
                # # else:
                # time.sleep(1.5)
                # arduino_device.current_steering = 20
                # time.sleep(0.05)
                # arduino_device.current_speed = -80
                # time.sleep(6)
                # arduino_device.current_steering = 10
                # #time.sleep((distance - lidar_device.STEP3_DISTANCE_OFFSET) * lidar_device.STEP3_BACK_PARAM)
                # time.sleep(2)
                # arduino_device.current_speed = 0
                # time.sleep(3)
                # # 주차 완료
                # lidar_device.parking_step += 1
                arduino_device.current_speed = 0
                time.sleep(.1)
                if distance < 1000:
                    arduino_device.current_steering = 0
                    time.sleep(.1)
                    arduino_device.current_speed = 50
                    time.sleep(2)
                    arduino_device.current_steering = 10
                    time.sleep(1)
                    arduino_device.current_steering = 20
                    time.sleep(2.1)
                    arduino_device.current_steering = 10
                    time.sleep(1)
                    arduino_device.current_speed = 0
                    time.sleep(1)
                    arduino_device.current_speed = -50
                    time.sleep(6.2)
                    arduino_device.current_speed = 0
                    time.sleep(0.5)
                arduino_device.current_speed = -50
                time.sleep(5)
                arduino_device.current_speed = 0
                time.sleep(.1)
                arduino_device.current_steering = 0
                time.sleep(.5)
                arduino_device.current_speed = 50
                time.sleep(3)
                arduino_device.current_speed = 0
                time.sleep(1)
                arduino_device.current_steering = 20
                time.sleep(.4)
                arduino_device.current_speed = -50
                time.sleep(9)
                arduino_device.current_speed = 0

                time.sleep(0.5)
                arduino_device.current_steering = 10
                time.sleep(0.5)
                arduino_device.current_speed = -50
                time.sleep(2)
                if distance > 1000:
                    time.sleep((distance - lidar_device.STEP3_DISTANCE_OFFSET) * lidar_device.STEP3_BACK_PARAM)
                arduino_device.current_speed = 0
                time.sleep(3)
                lidar_device.parking_step += 1
        elif lidar_device.parking_step == 4:
            # 차 빼기
            arduino_device.current_speed = 100
            time.sleep(1)
            arduino_device.current_steering = 20
            time.sleep(6.2)
            arduino_device.current_steering = 10
            time.sleep(10)
            arduino_device.current_speed = 0
            lidar_device.parking_step = 0

        time.sleep(0.1)



class MyApp(QWidget):
    def __init__(self):
        global main_ui
        super().__init__()
        self.initUI()
        main_ui = self
        self.driving_mode = ''
        self.driving_state = False
        self.record_starttime = time.time()
        self.main_cam_orig_writer, self.main_cam_infer_writer, self.sub_cam_orig_writer, self.lidar_image_writer = None, None, None, None
        self.steering_log, self.lidar_log = [], []
        self.recording = False

    def initUI(self):
        layout = QGridLayout()

        # Left top checkbox group
        self.leftTopGroup = QGroupBox('Modes / Manual Driving')
        leftTopLayout = QHBoxLayout()
        self.checkbox_drive = QCheckBox('DRIVE')
        self.checkbox_mission = QCheckBox('AV/TR')
        self.checkbox_park = QCheckBox('PARK')
        self.checkbox_test = QCheckBox('TEST')
        self.checkbox_manual = QCheckBox('MANUAL')
        self.checkbox_record = QCheckBox('RECORD')
        self.checkbox_drive.stateChanged.connect(lambda: self.modeChanged(self.checkbox_drive))
        self.checkbox_mission.stateChanged.connect(lambda: self.modeChanged(self.checkbox_mission))
        self.checkbox_park.stateChanged.connect(lambda: self.modeChanged(self.checkbox_park))
        self.checkbox_test.stateChanged.connect(lambda: self.modeChanged(self.checkbox_test))
        self.checkbox_manual.stateChanged.connect(lambda: self.modeChanged(self.checkbox_manual))
        self.checkbox_record.stateChanged.connect(self.recordChanged)
        modeLayout = QVBoxLayout()
        modeLayout.addWidget(self.checkbox_drive)
        modeLayout.addWidget(self.checkbox_mission)
        modeLayout.addWidget(self.checkbox_park)
        modeLayout.addWidget(self.checkbox_test)
        modeLayout.addWidget(self.checkbox_manual)
        modeLayout.addWidget(self.checkbox_record)

        manualLayout = QVBoxLayout()
        leftTopLayout.addLayout(modeLayout)
        leftTopLayout.addLayout(manualLayout)

        self.leftTopGroup.setLayout(leftTopLayout)
        
        # Right top driver select group
        self.rightTopGroup = QGroupBox('Device Select')
        rightTopLayout = QVBoxLayout()

        
        self.deviceReloadButton = QPushButton('Device Reload')
        self.deviceReloadButton.clicked.connect(self.deviceReloadButtonClicked)
        rightTopLayout.addWidget(self.deviceReloadButton)

        self.camComboBox = QComboBox()
        self.camSelectButton = QPushButton('Connect')
        self.createDeviceSelectionLayout(rightTopLayout, 'CAM Device', self.populateCamComboBox, self.camComboBox, self.camSelectButton, self.selectCamDevice)
        
        self.subcamComboBox = QComboBox()
        self.subcamSelectButton = QPushButton('Connect')
        self.createDeviceSelectionLayout(rightTopLayout, 'SCAM Device', self.populateCamComboBox, self.subcamComboBox, self.subcamSelectButton, self.selectSubCamDevice)

        self.lidarComboBox = QComboBox()
        self.lidarSelectButton = QPushButton('Connect')
        self.createDeviceSelectionLayout(rightTopLayout, 'Lidar Device', self.populateSerialComboBox, self.lidarComboBox, self.lidarSelectButton, self.selectLidarDevice)

        self.ardComboBox = QComboBox()
        self.ardSelectButton = QPushButton('Connect')
        self.createDeviceSelectionLayout(rightTopLayout, 'Ard Device', self.populateSerialComboBox, self.ardComboBox, self.ardSelectButton, self.selectArdDevice)

        self.rightTopGroup.setLayout(rightTopLayout)

        # Left bottom group
        self.leftBottomGroup = QGroupBox('Sensor Values')
        self.sensorLabel = QLabel('')
        leftBottomLayout = QVBoxLayout()
        leftBottomLayout.addWidget(self.sensorLabel)
        self.leftBottomGroup.setLayout(leftBottomLayout)

        # Right bottom group
        self.rightBottomGroup = QGroupBox('Control')
        self.StartButton = QPushButton('START')
        self.speed255Button = QPushButton('Speed : 255')
        self.speed150Button = QPushButton('Speed : 150')
        self.speed080Button = QPushButton('Speed : 80')
        self.speed000Button = QPushButton('Speed : 0')
        self.speedm150Button = QPushButton('Speed : -150')
        self.settingAngleButton = QPushButton('Car Angle Setting')
        self.updateCamParamButton = QPushButton('Update CAM Parameters')
        self.StartButton.clicked.connect(self.Start_Driving)
        self.speed255Button.clicked.connect(self.speed255ButtonClicked)
        self.speed150Button.clicked.connect(self.speed150ButtonClicked)
        self.speed080Button.clicked.connect(self.speed080ButtonClicked)
        self.speed000Button.clicked.connect(self.speed000ButtonClicked)
        self.speedm150Button.clicked.connect(self.speedm150ButtonClicked)
        self.settingAngleButton.clicked.connect(self.settingAngleButtonClicked)
        self.updateCamParamButton.clicked.connect(self.updateCamParamButtonClicked)
        rightBottomLayout = QVBoxLayout()
        rightBottomLayout.addWidget(self.StartButton)
        rightBottomLayout.addWidget(self.speed255Button)
        rightBottomLayout.addWidget(self.speed150Button)
        rightBottomLayout.addWidget(self.speed080Button)
        rightBottomLayout.addWidget(self.speed000Button)
        rightBottomLayout.addWidget(self.speedm150Button)
        rightBottomLayout.addWidget(self.settingAngleButton)
        rightBottomLayout.addWidget(self.updateCamParamButton)
        
        self.rightBottomGroup.setLayout(rightBottomLayout)

        layout.addWidget(self.leftTopGroup, 0, 0)
        layout.addWidget(self.rightTopGroup, 0, 1)
        layout.addWidget(self.leftBottomGroup, 1, 0)
        layout.addWidget(self.rightBottomGroup, 1, 1)

        self.setLayout(layout)
        self.setWindowTitle('Self Driving UI ver 1.0.5')
        self.setGeometry(300, 300, 800, 600)
        
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.updateImage)
        self.timer.start(100)  # Update the image every 100 ms
        self.show()

    def createDeviceSelectionLayout(self, parentLayout, labelText, populateFunc, comboBox, selectButton, selectFunc):
        hbox = QHBoxLayout()
        label = QLabel(labelText)
        populateFunc(comboBox)
        selectButton.clicked.connect(lambda: selectFunc(comboBox, selectButton))
        hbox.addWidget(label)
        hbox.addWidget(comboBox)
        hbox.addWidget(selectButton)
        parentLayout.addLayout(hbox)

    def populateCamComboBox(self, comboBox):
        index = 0
        while index < 10:
            cap = cv2.VideoCapture(index)
            if cap.read()[0]:
                comboBox.addItem(str(index))
            cap.release()
            index += 1

    def populateSerialComboBox(self, comboBox):
        comboBox.clear()
        ports = serial.tools.list_ports.comports()
        for port in ports:
            comboBox.addItem(port.device)

    def modeChanged(self, checkbox):
        if checkbox.isChecked():
            if checkbox == self.checkbox_drive:
                self.driving_mode = 'drive'
                self.checkbox_mission.setChecked(False)
                self.checkbox_park.setChecked(False)
                self.checkbox_test.setChecked(False)
                self.checkbox_manual.setChecked(False)
            elif checkbox == self.checkbox_mission:
                self.driving_mode = 'mission'
                self.checkbox_drive.setChecked(False)
                self.checkbox_park.setChecked(False)
                self.checkbox_test.setChecked(False)
                self.checkbox_manual.setChecked(False)
            elif checkbox == self.checkbox_park:
                self.driving_mode = 'park'
                self.checkbox_drive.setChecked(False)
                self.checkbox_mission.setChecked(False)
                self.checkbox_test.setChecked(False)
                self.checkbox_manual.setChecked(False)
            elif checkbox == self.checkbox_test:
                self.driving_mode = 'test'
                self.checkbox_drive.setChecked(False)
                self.checkbox_mission.setChecked(False)
                self.checkbox_park.setChecked(False)
                self.checkbox_manual.setChecked(False)
            elif checkbox == self.checkbox_manual:
                self.driving_mode = 'manual'
                self.checkbox_drive.setChecked(False)
                self.checkbox_mission.setChecked(False)
                self.checkbox_park.setChecked(False)
                self.checkbox_test.setChecked(False)
    
    def recordChanged(self):
        maincam_device.record = self.checkbox_record.isChecked()
        subcam_device.record = self.checkbox_record.isChecked()
        if self.checkbox_record.isChecked():
            self.record_starttime = time.time()
        else:
            self.record_starttime = 0

    def selectCamDevice(self, comboBox, selectButton):
        if maincam_device.cap is None:
            index = int(comboBox.currentText())
            comboBox.setEnabled(False)
            maincam_device.start_camera(index)
            selectButton.setText("Disconnect")
            #self.checkboxRecord.setEnabled(False)
        else:
            comboBox.setEnabled(True)
            maincam_device.stop_camera()
            selectButton.setText("Connect")
            #self.checkboxRecord.setEnabled(True)
            
    def selectSubCamDevice(self, comboBox, selectButton):
        if subcam_device.cap is None:
            index = int(comboBox.currentText())
            comboBox.setEnabled(False)
            subcam_device.start_camera(index)
            selectButton.setText("Disconnect")
            #self.checkboxRecord.setEnabled(False)
        else:
            comboBox.setEnabled(True)
            subcam_device.stop_camera()
            selectButton.setText("Connect")
            #self.checkboxRecord.setEnabled(False)

    def selectLidarDevice(self, comboBox, selectButton):
        if lidar_device.lidar_session is None:
            port = comboBox.currentText()
            comboBox.setEnabled(False)
            selectButton.setText("Disconnect")
            lidar_device.start_lidar(port)
        else:
            comboBox.setEnabled(True)
            lidar_device.stop_lidar()
            selectButton.setText("Connect")

    def selectArdDevice(self, comboBox, selectButton):
        if arduino_device.serial_session is None:
            port = comboBox.currentText()
            comboBox.setEnabled(False)
            selectButton.setText("Disconnect")
            arduino_device.start_arduino(port)
        else:
            comboBox.setEnabled(True)
            arduino_device.stop_arduino()
            selectButton.setText("Connect")

    def set_checkboxes_enabled(self, enabled):
        self.checkbox_drive.setEnabled(enabled)
        self.checkbox_mission.setEnabled(enabled)
        self.checkbox_park.setEnabled(enabled)
        self.checkbox_test.setEnabled(enabled)
        self.checkbox_manual.setEnabled(enabled)
        self.checkbox_record.setEnabled(enabled)

    def slow_stop(self):
        arduino_device.current_speed = 200
        time.sleep(0.2)
        arduino_device.current_speed = 150
        time.sleep(0.2)
        arduino_device.current_speed = 100
        time.sleep(0.2)
        arduino_device.current_speed = 50
        time.sleep(0.2)
        arduino_device.current_speed = 0

    def Start_Driving(self):
        if self.checkbox_drive.isChecked():
            if not self.driving_state:
                if maincam_device.cap is None or subcam_device.cap is None or arduino_device.serial_session is None:
                    QMessageBox.information(self, 'Warning', 'Please Connect MainCam, SubCam, Arduino.',
                                            QMessageBox.Ok)
                    return
                arduino_device.current_speed = 255
            else:
                self.slow_stop()
        elif self.checkbox_test.isChecked():
            if not self.driving_state:
                if maincam_device.cap != None:
                    return
                video_path = './test_video.mp4'
                maincam_device.start_video(video_path)
                if subcam_device.cap != None:
                    return
                video_path = './test_video_sub.mp4'
                subcam_device.start_video(video_path)
            else:
                maincam_device.stop_video()
                subcam_device.stop_video()
        elif self.checkbox_mission.isChecked():
            if not self.driving_state:
                if maincam_device.cap is None or subcam_device.cap is None or arduino_device.serial_session is None:
                    QMessageBox.information(self, 'Warning', 'Please Connect MainCam, SubCam, Arduino.',
                                            QMessageBox.Ok)
                    return
                arduino_device.current_speed = 255
                subcam_device.capture = False
                time.sleep(4.5)
                # subcam_device.avoid_state = True
                maincam_device.capture = False
                turn_left()
                maincam_device.capture = True
                time.sleep(2)
                subcam_device.capture = True
                subcam_device.avoid_state = True


            else:
                subcam_device.avoid_state = False
                subcam_device.traffic_state = False
                self.slow_stop()
        elif self.checkbox_park.isChecked():
            if not self.driving_state:
                parking()
            else:
                arduino_device.current_speed = 0
                pass
        elif self.checkbox_manual.isChecked():
            arduino_device.current_speed = 0
            arduino_device.current_steering = 10

        self.driving_state = not self.driving_state
        self.set_checkboxes_enabled(not self.driving_state)
        if self.driving_state:
            if self.checkbox_record.isChecked():
                self.start_record()
            self.StartButton.setText("STOP")
        else:
            if self.checkbox_record.isChecked():
                self.stop_record()
            self.StartButton.setText("START")

    def updateImage(self):
        main_image = np.zeros((720, 1280, 3), np.uint8)
        main_image[:360, :1280] = maincam_device.update_img
        main_image[360:, :640] = subcam_device.update_img
        main_image[360:, 640:1000] = lidar_device.update_img
        if self.recording:
            self.checkbox_record.setText(f"RECORDING '{time.strftime('%M:%S', time.localtime(time.time() - self.record_starttime))}'")
            if maincam_device.cap is not None:
                self.main_cam_orig_writer.write(main_image[:360, :640])
                self.main_cam_infer_writer.write(main_image[:360, 640:])
            if subcam_device.cap is not None:
                self.sub_cam_orig_writer.write(main_image[360:, :640])
            if lidar_device.lidar_session is not None:
                self.lidar_image_writer.write(main_image[360:, 640:1000])
                self.lidar_log.append(lidar_device.scan_data)
            self.steering_log.append(arduino_device.current_steering)

        else:
            self.checkbox_record.setText("RECORD")
        self.showImage(main_image)

    def showImage(self, img):
        qformat = QImage.Format_Indexed8
        if len(img.shape) == 3:
            if img.shape[2] == 4:
                qformat = QImage.Format_RGBA8888
            else:
                qformat = QImage.Format_RGB888
        img = QImage(img, img.shape[1], img.shape[0], img.strides[0], qformat)
        img = img.rgbSwapped()
        self.sensorLabel.setPixmap(QPixmap.fromImage(img))
        self.sensorLabel.setScaledContents(True)
        
    def speed255ButtonClicked(self):
        arduino_device.current_speed = 255
        
    def speed150ButtonClicked(self):
        arduino_device.current_speed = 150
        
    def speed080ButtonClicked(self):
        arduino_device.current_speed = 80
        
    def speed000ButtonClicked(self):
        arduino_device.current_speed = 0
    
    def speedm150ButtonClicked(self):
        arduino_device.current_speed = -150
        
    def settingAngleButtonClicked(self):
        arduino_device.send_data('setting;')
        
    def deviceReloadButtonClicked(self):
        # commands = [
        #     "echo 'dlwodls8747' | sudo -S chmod 777 /dev/ttyUSB*",
        #     "echo 'dlwodls8747' | sudo -S chmod 777 /dev/ttyACM*"
        # ]
        # # 각 명령어 실행
        # for command in commands:
        #     result = subprocess.run(command, shell=True, capture_output=True, text=True)
        #     print(result.stdout)
        #     if result.returncode != 0:
        #         print(f"Error: {result.stderr}")
        self.populateCamComboBox(self.camComboBox)
        self.populateSerialComboBox(self.ardComboBox)
        self.populateSerialComboBox(self.lidarComboBox)
    
    def updateCamParamButtonClicked(self):
        maincam_device.load_params()
        subcam_device.load_params()



    def keyPressEvent(self, event):
        key = event.key()
        if key == Qt.Key_M:
            if self.checkbox_drive.isEnabled():
                if self.checkbox_drive.isChecked():
                    self.checkbox_drive.setChecked(False)
                    self.checkbox_mission.setChecked(True)
                elif self.checkbox_mission.isChecked():
                    self.checkbox_mission.setChecked(False)
                    self.checkbox_park.setChecked(True)
                elif self.checkbox_park.isChecked():
                    self.checkbox_park.setChecked(False)
                    self.checkbox_test.setChecked(True)
                elif self.checkbox_test.isChecked():
                    self.checkbox_test.setChecked(False)
                    self.checkbox_manual.setChecked(True)
                elif self.checkbox_manual.isChecked():
                    self.driving_mode = ''
                    self.checkbox_manual.setChecked(False)
                else:
                    self.checkbox_drive.setChecked(True)
        if key == Qt.Key_R:
            if self.checkbox_record.isEnabled():
                self.checkbox_record.setChecked(not self.checkbox_record.isChecked())
        if key == Qt.Key_S:
            self.Start_Driving()
        if key == Qt.Key_Escape:
            self.close()
        if self.driving_state and self.checkbox_manual.isChecked():
            if key == Qt.Key_Up:
                if arduino_device.current_speed >= 250:
                    arduino_device.current_speed = 255
                else:
                    arduino_device.current_speed += 5
            if key == Qt.Key_Down:
                if arduino_device.current_speed <= -250:
                    arduino_device.current_speed = -255
                else:
                    arduino_device.current_speed -= 5
            if key == Qt.Key_Right:
                if arduino_device.current_steering >= 19:
                    arduino_device.current_steering = 20
                else:
                    arduino_device.current_steering += 1
            if key == Qt.Key_Left:
                if arduino_device.current_steering <= 1:
                    arduino_device.current_steering = 0
                else:
                    arduino_device.current_steering -= 1

    def closeEvent(self, event):
        global main_ui
        main_ui = None

    def start_record(self):
        self.record_starttime = time.time()
        self.record_formatted_time = time.strftime('%Y-%m-%d-%H%M%S', time.localtime(self.record_starttime))
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        os.mkdir(f'./records/{self.record_formatted_time}')
        self.main_cam_orig_writer = cv2.VideoWriter(f'./records/{self.record_formatted_time}/{self.record_formatted_time}_main.mp4', fourcc, 10.0, (640, 360))
        self.main_cam_infer_writer = cv2.VideoWriter(f'./records/{self.record_formatted_time}/{self.record_formatted_time}_infer.mp4', fourcc, 10.0, (640, 360))
        self.sub_cam_orig_writer = cv2.VideoWriter(f'./records/{self.record_formatted_time}/{self.record_formatted_time}_sub.mp4', fourcc, 10.0, (640, 360))
        self.lidar_image_writer = cv2.VideoWriter(f'./records/{self.record_formatted_time}/{self.record_formatted_time}_lidar.mp4', fourcc, 10.0, (360, 360))
        self.recording = True

    def stop_record(self):
        self.recording = False
        with open(f'./records/{self.record_formatted_time}/{self.record_formatted_time}_steering.json', 'w') as f:
            json.dump(self.steering_log, f)
        with open(f'./records/{self.record_formatted_time}/{self.record_formatted_time}_lidar.json', 'w') as f:
            json.dump(self.lidar_log, f)
        self.main_cam_orig_writer.release()
        self.main_cam_infer_writer.release()
        self.sub_cam_orig_writer.release()
        self.lidar_image_writer.release()


def avoid_car_speed150():
    time.sleep(0.1)
    arduino_device.current_steering = 0
    time.sleep(0.1)
    arduino_device.current_speed = 150
    time.sleep(1.3)
    arduino_device.current_steering = 20
    time.sleep(1.9)
    arduino_device.current_steering = 10
    time.sleep(0.5)
    arduino_device.current_steering = 20
    time.sleep(1.3)
    arduino_device.current_steering = 0
    time.sleep(1.1)
    arduino_device.current_steering = 10
    time.sleep(0.1)
    arduino_device.current_speed = 170


def turn_left():
    time.sleep(0.1)  # originally, 0.05
    arduino_device.current_steering = 0
    time.sleep(0.9)
    arduino_device.current_steering = 10
    maincam_device.line_name = 'llline'



def turn_right():
    time.sleep(0.1)  # originally, 0.05
    arduino_device.current_steering = 20
    time.sleep(0.7)
    arduino_device.current_steering = 10
    maincam_device.line_name = 'rrline'


def avoid_car_speed255():
    time.sleep(0.1)
    arduino_device.current_steering = 0
    time.sleep(0.1)
    arduino_device.current_speed = 255
    time.sleep(0.6)
    arduino_device.current_steering = 20
    time.sleep(1.2)
    arduino_device.current_steering = 10
    time.sleep(0.45)
    # 원래 버전
    arduino_device.current_steering = 20
    time.sleep(0.65)
    # arduino_device.current_steering = 0
    # time.sleep(1.0)
    # arduino_device.current_steering = 10
    #time.sleep(0.1)
    arduino_device.current_speed = 170


def subcam_task():
    if subcam_device.cap is not None and subcam_device.capture:
        ret, frame = subcam_device.cap.read()
        if ret:
            t1 = time.time_ns()
            results = subcam_device.model(frame, device='mps', conf=0.6, verbose=False)
            subcam_device.update_img = results[0].plot()
            for idx, box in enumerate(results[0].boxes):
                if int(box.cls) == 1 and subcam_device.traffic_state:
                    traffic_size = int(box.xyxy[0][2] - box.xyxy[0][0]) * int(box.xyxy[0][3] - box.xyxy[0][1])
                    traffic_point_y = int(box.xyxy[0][3] + box.xyxy[0][1]) // 2
                    traffic_width = int(box.xyxy[0][2] - box.xyxy[0][0])
                    traffic_r_x = int(box.xyxy[0][0]) + int(traffic_width // 6)
                    traffic_y_x = int(box.xyxy[0][0]) + int(3 * traffic_width // 6)
                    traffic_g_x = int(box.xyxy[0][0]) + int(5 * traffic_width // 6)
                    print("[find traffic sign]")
                    print(f"R : {[frame[traffic_point_y][traffic_r_x]]}")
                    print(f"Y : {[frame[traffic_point_y][traffic_y_x]]}")
                    print(f"G : {[frame[traffic_point_y][traffic_g_x]]}")
                    color_green = [frame[traffic_point_y][traffic_g_x]]
                    if traffic_size > 17000:
                        # arduino_device.current_speed = 0
                        green_color = int(int(color_green[0][0]) + int(color_green[0][1]) + int(color_green[0][2])) // 3
                        print(green_color)
                        if green_color > 230:
                            arduino_device.current_speed = 150
                            subcam_device.traffic_state = False
                        else:
                            arduino_device.current_speed = 0

                if int(box.cls) == 0 and subcam_device.avoid_state:
                    car_size = int(box.xyxy[0][2] - box.xyxy[0][0]) * int(box.xyxy[0][3] - box.xyxy[0][1])
                    print(f'car_size : {car_size}')
                    if car_size > 6000:  # originally, 6000
                        averagex = int((box.xyxy[0][0] + box.xyxy[0][2]) / 2)
                        # 중심점이 왼쪽이면 무시, 가운데쯤 있으면 회피기동
                        if 220 <= averagex <= 480:
                            # maincam_device.capture = False
                            # avoid_car_speed255()
                            # maincam_device.capture = True
                            # subcam_device.avoid_state = False
                            # subcam_device.traffic_state = True
                            # arduino_device.current_speed = 0
                            subcam_device.avoid_state = False
                            maincam_device.capture = False
                            turn_right()
                            maincam_device.capture = True
                            time.sleep(3)
                            arduino_device.current_speed = 150
                            maincam_device.angle_max = int(maincam_device.angle_max * 0.95)
                            maincam_device.angle_min = int(maincam_device.angle_min * 0.95)
                            subcam_device.traffic_state = True

                            # if maincam_device.line_name == 'rrline':
                            #     subcam_device.found_num += 1
                            #     subcam_device.capture = False
                            #     maincam_device.capture = False
                            #     turn_left()
                            #     maincam_device.line_name = 'llline'
                            #     maincam_device.capture = True
                            #     time.sleep(0.5)
                            #     subcam_device.capture = True
                            # elif maincam_device.line_name == 'llline':
                            #     subcam_device.found_num += 1
                            #     subcam_device.capture = False
                            #     maincam_device.capture = False
                            #     turn_right()
                            #     maincam_device.line_name = 'rrline'
                            #     maincam_device.capture = True
                            #     time.sleep(0.5)
                            #     subcam_device.capture = True
                            # if subcam_device.found_num == 2:
                            #     subcam_device.avoid_state = False
                            #     subcam_device.traffic_state = True
                            #     subcam_device.found_num = 0
                            #
                            break
            # if subcam_device.record:
            #     subcam_device.writer_orig.write(frame)
            t2 = time.time_ns()
            #print(f'inference time : {(t2 - t1) / 1e+6}ms')
            if subcam_device.update_img.shape[0] != 360 or subcam_device.update_img.shape[1] != 640:
                subcam_device.update_img = cv2.resize(subcam_device.update_img, (640, 360))

    global subcam_timer
    if main_ui is not None:
        subcam_timer = threading.Timer(1 / subcam_device.fps, subcam_task)
        subcam_timer.start()


def maincam_task():
    if maincam_device.cap is not None:
        ret, frame = maincam_device.cap.read()
        if ret:
            try:
                t1 = time.time_ns()
                img_inferenced, drawed_img, line1_ang, line2_ang, mid_bias = inf_angle_mainline(maincam_device.model, frame, maincam_device.SAMPLING_RATE, maincam_device.bev_width_offset, maincam_device.bev_height_offset, maincam_device.line_name, maincam_device.poly_degree)
                steering_value = 9999.0
                first = line1_ang
                second = line2_ang
                if maincam_device.line_name == 'llline':
                    first = line2_ang
                    second = line1_ang
                if first is not None:
                    idx, steering_value = map_to_steering(first, maincam_device.angle_min, maincam_device.angle_max, maincam_device.steering_values)
                    if maincam_device.save_statistics:
                        maincam_device.statistics_list.append([idx, steering_value])
                    if maincam_device.capture:
                        if mid_bias is not None:
                            arduino_device.current_steering = steering_value + int(mid_bias)
                        else:
                            arduino_device.current_steering = steering_value
                elif second is not None:
                    idx, steering_value = map_to_steering(second, maincam_device.angle_min, maincam_device.angle_max, maincam_device.steering_values)
                    if maincam_device.save_statistics:
                        maincam_device.statistics_list.append([idx, steering_value])
                    if maincam_device.capture:
                        if mid_bias is not None:
                            arduino_device.current_steering = steering_value + int(mid_bias)
                        else:
                            arduino_device.current_steering = steering_value
                if maincam_device.verbose:
                    drawed_img = put_message(drawed_img, 3, [f'rl_ang : {line1_ang}', f'll_ang : {line2_ang}', f'steer : {steering_value}'])
                # if maincam_device.record:
                #     maincam_device.writer_orig.write(frame)
                #     maincam_device.writer_inferenced.write(drawed_img)
                if frame.shape[0] != 360 or frame.shape[1] != 640:
                    frame = cv2.resize(frame, (640, 360))
                if drawed_img.shape[0] != 360 or drawed_img.shape[1] != 640:
                    drawed_img = cv2.resize(drawed_img, (640, 360))
                maincam_device.update_img[:360, :640] = img_inferenced
                maincam_device.update_img[:360, 640:] = drawed_img
                t3 = time.time_ns()
                print(f'inference time : {(t3 - t1) / 1e+6}ms')
            except Exception as e:
                print(f"An error occured: {e}")
                traceback.print_exc()
    global maincam_timer
    if main_ui is not None:
        maincam_timer = threading.Timer(1 / maincam_device.fps, maincam_task)
        maincam_timer.start()


def arduino_task():
    global arduino_device
    while True:
        if arduino_device.current_speed != arduino_device.before_speed:
            arduino_device.before_speed = arduino_device.current_speed
            if arduino_device.current_speed >= 0:
                arduino_device.send_data(f'speed+{arduino_device.current_speed:03d};')
            else:
                arduino_device.send_data(f'speed-{abs(arduino_device.current_speed):03d};')
            print(f'speed value sended : {arduino_device.current_speed}')
        elif arduino_device.current_steering != arduino_device.before_steering:
            arduino_device.before_steering = arduino_device.current_steering
            arduino_device.send_data(f'angle{arduino_device.current_steering:02d};')
            print(f'steering value sended : {arduino_device.current_steering}')
        time.sleep(0.01)


if __name__ == '__main__':
    subcam_device = subcam()
    subcam_device.load_params()
    subcam_timer = threading.Timer(1 / subcam_device.fps, subcam_task)

    maincam_device = maincam()
    maincam_device.load_params()
    maincam_timer = threading.Timer(1 / maincam_device.fps, maincam_task)

    lidar_device = lidar()
    lidar_thread = threading.Thread(target=lidar_scan, daemon=True)

    arduino_device = arduino()
    arduino_thread = threading.Thread(target=arduino_task, daemon=True)

    app = QApplication(sys.argv)
    ex = MyApp()

    subcam_timer.start()
    maincam_timer.start()
    arduino_thread.start()
    lidar_thread.start()

    sys.exit(app.exec_())
    lidar_thread.join()
    arduino_thread.join()