import sys
import cv2
from PyQt5.QtWidgets import QMessageBox, QApplication, QWidget, QVBoxLayout, QHBoxLayout, QGroupBox, QCheckBox, QLabel, \
    QComboBox, QPushButton, QGridLayout
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
        self.model_car = YOLO('./models/yolov10x.pt', task='detect')
        self.model_traffic = YOLO('./models/yolov8m-sub-ep200-frz-d2.pt', task='detect')
        self.model_car.to('mps')
        self.model_traffic.to('mps')
        self.record = False
        self.writer_orig = None
        self.writer_inferenced = None
        self.update_img = np.zeros((360, 640, 3), dtype=np.uint8)
        self.steering_values = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20]
        self.fps = 5.0
        self.capture = True
        self.avoid_state = False
        self.cross_state = False
        self.traffic_state = False
        self.model_car(np.zeros((360, 640, 3), dtype=np.uint8), conf=0.2)
        self.model_traffic(np.zeros((360, 640, 3), dtype=np.uint8), conf=0.2)
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
            self.writer_inferenced = cv2.VideoWriter(f'./videos/inferenced/{formatted_time}.mp4', fourcc, self.fps,
                                                     (640, 360))
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
        self.model.half()
        self.model.to('mps')
        self.record = False
        self.save_statistics = True
        self.SAMPLING_RATE = 0.8
        self.bev_height_offset = 0.2
        self.bev_width_offset = 0.27
        self.update_img = np.zeros((360, 1280, 3), dtype=np.uint8)
        self.angle_min = -80
        self.angle_max = 80
        self.steering_values = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20]
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

        self.BACK_PARAM_1 = 0.015
        self.BACK_PARAM_2 = 0.014
        self.BACK_PARAM_3 = 0.016
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


def parking():
    lidar_device.parking_step = 1
    while lidar_device.parking_step > 0:
        if lidar_device.parking_step == 1:
            # 1단계. 직진하면서 차량 두대 발견 -> 비틀림 각도 산출
            arduino_device.current_speed = 80
            arduino_device.current_steering = 10
            if len(lidar_device.scan_data) < 10:
                time.sleep(0.1)
                continue
            distance, lidar_car = lidar_analyze(lidar_device.scan_data)  # 여기 car detect 함수 만들어서, 우측 차량과 거리 검출
            if lidar_car is not None:
                if lidar_device.car_a is None:
                    lidar_device.car_a_dist, lidar_device.car_a = distance, lidar_car
                    print(f'[STEP 1] Car A Detected : distance is {distance}mm')
                    time.sleep(2)
                elif lidar_device.car_b is None:
                    arduino_device.current_speed = 0
                    lidar_device.car_b_dist, lidar_device.car_b = distance, lidar_car
                    print(f'[STEP 1] Car B Detected : distance is {distance}mm')
                    y_diff = lidar_device.car_b_dist - lidar_device.car_a_dist
                    lidar_device.turn_angle = math.atan(
                        -1 * y_diff / 100) * 180 / math.pi  # 비틀림 각도 산출 (car_a의 가장 좌측, car_b의 가장 좌측 좌표를 통해)
                    print(f'[STEP 1] Error Angle : {lidar_device.turn_angle} degree')
                    lidar_device.parking_step = 2  # 2단계(후진으로 각도 보정)으로 넘어감
                    time.sleep(2)
        elif lidar_device.parking_step == 2:
            abs_angle = abs(lidar_device.turn_angle)
            if abs_angle > 10:  # 어느정도는 넘어야 보정함
                if lidar_device.turn_angle > 0:  # 차량이 우측으로 치우쳐짐
                    arduino_device.current_steering = 20
                else:  # 차량이 좌측으로 치우쳐짐
                    arduino_device.current_steering = 0
                time.sleep(0.1)
                arduino_device.current_speed = -80
                if abs_angle < 25:  # abs_turn_angle : 10 ~ 25
                    correction_time = abs_angle * lidar_device.BACK_PARAM_1
                elif abs_angle < 50:  # abs_turn_angle : 25 ~ 50
                    correction_time = abs_angle * lidar_device.BACK_PARAM_2
                else:
                    correction_time = abs_angle * lidar_device.BACK_PARAM_3
                print(f'[STEP 2] Angle Correction : {correction_time} seconds')
                time.sleep(correction_time)  # 각도 보정 완료
                arduino_device.current_speed = 0
            arduino_device.current_steering = 10
            print(f'[STEP 2] Back for 1.5s')
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
                print(f'[STEP 3] Car Detected : distance is {distance}mm')
                arduino_device.current_speed = 0
                time.sleep(.1)
                if distance < 1000:
                    parking_20cm_far()
                arduino_device.current_speed = -50
                time.sleep(5)
                arduino_device.current_speed = 0
                time.sleep(.1)
                parking_90turn()
                # 이부분 알고리즘 수정 필요
                arduino_device.current_steering = 10
                time.sleep(0.5)
                arduino_device.current_speed = -50
                time.sleep(2)
                # 이부분 알고리즘 수정 필요
                if distance > 1000:
                    time.sleep((distance - lidar_device.STEP3_DISTANCE_OFFSET) * lidar_device.STEP3_BACK_PARAM)
                else:
                    time.sleep(1.5)
                arduino_device.current_speed = 0
                time.sleep(3)
                lidar_device.parking_step += 1
        elif lidar_device.parking_step == 4:
            # 차 빼기
            arduino_device.current_speed = 100
            time.sleep(1)
            arduino_device.current_steering = 20
            time.sleep(5.7)
            arduino_device.current_steering = 10
            time.sleep(10)
            arduino_device.current_speed = 0
            lidar_device.parking_step = 0

        time.sleep(0.1)


def parking_20cm_far():
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


def parking_90turn():
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
        '''
        UI의 레이아웃을 만들고, 기능을 연결하는 함수
        '''
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
        self.createDeviceSelectionLayout(rightTopLayout, 'CAM Device', self.populateCamComboBox, self.camComboBox,
                                         self.camSelectButton, self.selectCamDevice)

        self.subcamComboBox = QComboBox()
        self.subcamSelectButton = QPushButton('Connect')
        self.createDeviceSelectionLayout(rightTopLayout, 'SCAM Device', self.populateCamComboBox, self.subcamComboBox,
                                         self.subcamSelectButton, self.selectSubCamDevice)

        self.lidarComboBox = QComboBox()
        self.lidarSelectButton = QPushButton('Connect')
        self.createDeviceSelectionLayout(rightTopLayout, 'Lidar Device', self.populateSerialComboBox,
                                         self.lidarComboBox, self.lidarSelectButton, self.selectLidarDevice)

        self.ardComboBox = QComboBox()
        self.ardSelectButton = QPushButton('Connect')
        self.createDeviceSelectionLayout(rightTopLayout, 'Ard Device', self.populateSerialComboBox, self.ardComboBox,
                                         self.ardSelectButton, self.selectArdDevice)

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
        '''
        Device Selection Layout (1Label:device_type, 1Combobox:device_list, 1Button:device_select) 을 만드는 함수
        parentLayout : master UI의 우측상단 QVBoxLayout
        labelText : label에 들어갈 이름
        populateFunc : combobox에 해당 device들 찾아서 입력
        comboBox, selectButton : 각 combobox, selectbutton의 객체를 넣어주면 됨
        selectFunc : 선택될 때 Button과 연결되는 함수
        '''
        hbox = QHBoxLayout()
        label = QLabel(labelText)
        populateFunc(comboBox)
        selectButton.clicked.connect(lambda: selectFunc(comboBox, selectButton))
        hbox.addWidget(label)
        hbox.addWidget(comboBox)
        hbox.addWidget(selectButton)
        parentLayout.addLayout(hbox)

    def populateCamComboBox(self, comboBox):
        '''
        카메라 Combobox를 다시 불러올때 사용하는 함수
        '''
        comboBox.clear()
        index = 0
        while index < 10:
            cap = cv2.VideoCapture(index)
            if cap.read()[0]:
                comboBox.addItem(str(index))
            cap.release()
            index += 1

    def populateSerialComboBox(self, comboBox):
        '''
        시리얼 Combobox를 불러올 때 사용
        '''
        comboBox.clear()
        ports = serial.tools.list_ports.comports()
        for port in ports:
            comboBox.addItem(port.device)

    def modeChanged(self, checkbox):
        '''
        modeChanged를 변경할 때 (checkbox들을 왔다갔다 할 때) 모드를 정해주고, complement하게 작동하도록 하는 함수
        '''
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
        '''
        녹화기능을 활성/비활성화 시킬 때 사용하는 함수
        '''
        maincam_device.record = self.checkbox_record.isChecked()
        subcam_device.record = self.checkbox_record.isChecked()
        if self.checkbox_record.isChecked():
            self.record_starttime = time.time()
        else:
            self.record_starttime = 0

    def selectCamDevice(self, comboBox, selectButton):
        '''
        메인 카메라 선택 버튼을 눌렀을 때 실행되는 함수
        '''
        if maincam_device.cap is None:
            index = int(comboBox.currentText())
            comboBox.setEnabled(False)
            maincam_device.start_camera(index)
            selectButton.setText("Disconnect")
            # self.checkboxRecord.setEnabled(False)
        else:
            comboBox.setEnabled(True)
            maincam_device.stop_camera()
            selectButton.setText("Connect")
            # self.checkboxRecord.setEnabled(True)

    def selectSubCamDevice(self, comboBox, selectButton):
        '''
        서브 카메라 선택 버튼을 눌렀을 때 실행되는 함수
        '''
        if subcam_device.cap is None:
            index = int(comboBox.currentText())
            comboBox.setEnabled(False)
            subcam_device.start_camera(index)
            selectButton.setText("Disconnect")
            # self.checkboxRecord.setEnabled(False)
        else:
            comboBox.setEnabled(True)
            subcam_device.stop_camera()
            selectButton.setText("Connect")
            # self.checkboxRecord.setEnabled(False)

    def selectLidarDevice(self, comboBox, selectButton):
        '''
        라이다 기기 선택 버튼을 눌렀을 때 실행되는 함수
        '''
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
        '''
        아두이노 기기 선택 버튼을 눌렀을 때 실행되는 함수
        '''
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
        '''
        기능이 실행/종료될 때 실행시킴으로써 checkbox들을 활성/비활성화
        '''
        self.checkbox_drive.setEnabled(enabled)
        self.checkbox_mission.setEnabled(enabled)
        self.checkbox_park.setEnabled(enabled)
        self.checkbox_test.setEnabled(enabled)
        self.checkbox_manual.setEnabled(enabled)
        self.checkbox_record.setEnabled(enabled)

    def slow_stop(self):
        '''
        차량 천천히 종료되는 함수
        '''
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
        '''
        Start/Stop Button을 누르거나, keyboard S를 눌렀을 때 실행되는 함수
        '''
        if self.checkbox_drive.isChecked():
            '''
            주행 모드 : MainCam, Subcam, Arduino 모두 활성화 되어 있지 않으면 경고창, 활성화 시 속도 255 / slow_stop
            '''
            if not self.driving_state:
                if maincam_device.cap is None or subcam_device.cap is None or arduino_device.serial_session is None:
                    QMessageBox.information(self, 'Warning', 'Please Connect MainCam, SubCam, Arduino.',
                                            QMessageBox.Ok)
                    return
                maincam_device.capture = True
                subcam_device.capture = True
                maincam_device.line_name = 'rrline'
                arduino_device.current_speed = 255
            else:
                self.slow_stop()
        elif self.checkbox_test.isChecked():
            '''
            테스트 모드 : MainCam, Subcam에 각 경로에 있는 영상을 카메라에 입력으로 주고 테스트
            '''
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
            '''
            미션 모드
            순서
            STEP 1 : 5.5초간 서브카메라 비활성화
            STEP 2 : 메인카메라 비활성화 -> 좌측 라인 변경 -> 메인카메라 활성화 (좌측라인으로 달림)
            STEP 3 : 2초간 가만있고 서브카메라 / 장애물 회피 활성화
            '''
            if not self.driving_state:
                if maincam_device.cap is None or subcam_device.cap is None or arduino_device.serial_session is None:
                    QMessageBox.information(self, 'Warning', 'Please Connect MainCam, SubCam, Arduino.',
                                            QMessageBox.Ok)
                    return

                # [ STEP 1 ]
                maincam_device.capture = True
                subcam_device.capture = False
                subcam_device.avoid_state = False
                subcam_device.traffic_state = False
                subcam_device.cross_state = False
                arduino_device.current_speed = 200
                time.sleep(9 * .75)  # 첫 장애물로 인한 대기 시간 + 0.8

                # [ STEP 2 ]
                turn_left() # -1.2

                # [ STEP 3 ]
                time.sleep(1.5 * .75)  # 세번째 차량 찾기 시작하는 시간. 돌다가 주차된 차들을 찾거나, 너무 빨리 찾거나, 너무 늦게 찾을시 이 시간 수정도 고려
                subcam_device.capture = True
                subcam_device.avoid_state = True
            else:
                maincam_device.capture = True
                subcam_device.capture = True
                subcam_device.avoid_state = False
                subcam_device.traffic_state = False
                subcam_device.cross_state = False
                maincam_device.line_name = 'rrline'

                self.slow_stop()
        elif self.checkbox_park.isChecked():
            if not self.driving_state:
                parking_thread = threading.Thread(target=parking, daemon=True)
                parking_thread.start()
            else:
                arduino_device.current_speed = 0
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
            self.checkbox_record.setText(
                f"RECORDING '{time.strftime('%M:%S', time.localtime(time.time() - self.record_starttime))}'")
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
        self.main_cam_orig_writer = cv2.VideoWriter(
            f'./records/{self.record_formatted_time}/{self.record_formatted_time}_main.mp4', fourcc, 10.0, (640, 360))
        self.main_cam_infer_writer = cv2.VideoWriter(
            f'./records/{self.record_formatted_time}/{self.record_formatted_time}_infer.mp4', fourcc, 10.0, (640, 360))
        self.sub_cam_orig_writer = cv2.VideoWriter(
            f'./records/{self.record_formatted_time}/{self.record_formatted_time}_sub.mp4', fourcc, 10.0, (640, 360))
        self.lidar_image_writer = cv2.VideoWriter(
            f'./records/{self.record_formatted_time}/{self.record_formatted_time}_lidar.mp4', fourcc, 10.0, (360, 360))
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


def turn_left():
    '''
    왼쪽차선으로 갈아탈 때 사용하는 함수
    메인 카메라 끔 -> 좌측 스티어링 0.9s -> line_name 변경 -> 메인 카메라 켬
    '''
    maincam_device.capture = False
    time.sleep(0.1)  # originally, 0.05
    arduino_device.current_steering = 0
    time.sleep(1.7 * .75)
    arduino_device.current_steering = 17
    time.sleep(0.81 * .75)
    arduino_device.current_steering = 10
    maincam_device.line_name = 'llline'
    maincam_device.capture = True


def turn_right(car_bias=0):
    '''
    우측차선으로 갈아탈 때 사용하는 함수
    메인 카메라 끔 -> 우측 스티어링 0.7s -> line_name 변경 -> 메인 카메라 켬
    '''
    CAR_BIAS_PARAM = 0.003333 * 1.7
    maincam_device.capture = False
    time.sleep(0.1)  # originally, 0.05
    arduino_device.current_steering = 18
    time.sleep(1.2)
    arduino_device.current_steering = 0
    time.sleep(1)
    maincam_device.line_name = 'rrline'
    maincam_device.capture = True
    time.sleep(0.1)


def sign_brightness(area):
    h, w, c = area.shape
    width_offset = int(w * 0.15 / 3)
    red_box = area[
              int(h * (1 / 3)):int(h * (2 / 3)),
              0 + width_offset : int(w * 1 / 3 - width_offset)]

    yellow_box = area[
                 int(h * (1 / 3)):int(h * (2 / 3)),
                 int(w * 1 / 3) + width_offset:int(w * 2 / 3) - width_offset]
    green_box = area[
                int(h * (1 / 3)):int(h * (2 / 3)),
                int(w * 2 / 3) + width_offset:w - width_offset]
    red_hsv = cv2.cvtColor(red_box, cv2.COLOR_BGR2HSV)
    yellow_hsv = cv2.cvtColor(yellow_box, cv2.COLOR_BGR2HSV)
    green_hsv = cv2.cvtColor(green_box, cv2.COLOR_BGR2HSV)

    red_h = np.average(red_hsv[:, :, 0])
    yellow_h = np.average(yellow_hsv[:, :, 0])
    green_h = np.average(green_hsv[:, :, 0])

    red_s = np.average(red_hsv[:, :, 1])
    yellow_s = np.average(yellow_hsv[:, :, 1])
    green_s = np.average(green_hsv[:, :, 1])
    red_v = np.average(red_hsv[:, :, 2])
    yellow_v = np.average(yellow_hsv[:, :, 2])
    green_v = np.average(green_hsv[:, :, 2])

    print(f'red_h_value : {red_h}')
    print(f'yellow_h_avg : {yellow_h}')
    print(f'green_h_avg : {green_h}')

    print(f'red_s_value : {red_s}')
    print(f'yellow_s_avg : {yellow_s}')
    print(f'green_s_avg : {green_s}')

    print(f'red_v_value : {red_v}')
    print(f'yellow_v_avg : {yellow_v}')
    print(f'green_v_avg : {green_v}')
    return [[red_h, red_s, red_v], [yellow_h, yellow_s, yellow_v], [green_h, green_s, green_v]]

def subcam_task():
    if subcam_device.cap is not None:
        ret, frame = subcam_device.cap.read()
        # subcam_device.capture = True
        if ret:
            t1 = time.time_ns()
            # if subcam_device.traffic_state or subcam_device.cross_state:
            #     results = subcam_device.model_traffic(frame, device='mps', conf=0.6, verbose=False)
            # else:
            #     results = subcam_device.model_car(frame, device='mps', conf=0.2, verbose=False)


            if subcam_device.avoid_state:
                results = subcam_device.model_car(frame, device='mps', conf=0.2, verbose=False)
            else:
                results = subcam_device.model_traffic(frame, device='mps', conf=0.4, verbose=False)
            subcam_device.update_img = results[0].plot()
            if subcam_device.capture:
                for idx, box in enumerate(results[0].boxes):
                    if int(box.cls) == 7:
                        car_size = int(box.xyxy[0][2] - box.xyxy[0][0]) * int(box.xyxy[0][3] - box.xyxy[0][1])
                        car_width = int(box.xyxy[0][2] - box.xyxy[0][0])
                        averagex = int((box.xyxy[0][0] + box.xyxy[0][2]) / 2)
                        print(f'truck, size : {car_size}, width : {car_width}, averagex : {averagex}')
                    if int(box.cls) == 2:
                        car_size = int(box.xyxy[0][2] - box.xyxy[0][0]) * int(box.xyxy[0][3] - box.xyxy[0][1])
                        car_width = int(box.xyxy[0][2] - box.xyxy[0][0])
                        averagex = int((box.xyxy[0][0] + box.xyxy[0][2]) / 2)
                        print(f'car, size : {car_size}, width : {car_width}, averagex : {averagex}')
                    if int(box.cls) == 3:
                        car_size = int(box.xyxy[0][2] - box.xyxy[0][0]) * int(box.xyxy[0][3] - box.xyxy[0][1])
                        car_width = int(box.xyxy[0][2] - box.xyxy[0][0])
                        averagex = int((box.xyxy[0][0] + box.xyxy[0][2]) / 2)
                        print(f'motorcycle, size : {car_size}, width : {car_width}, averagex : {averagex}')
                    if int(box.cls) == 2 and subcam_device.cross_state:
                        cross_walk_width = int(box.xyxy[0][2] - box.xyxy[0][0])
                        cross_walk_height = int(box.xyxy[0][3] - box.xyxy[0][1])
                        cross_walk_upline = int(box.xyxy[0][1])
                        print(cross_walk_upline)

                        # box 좌표로 판단하는 기준 만들고, 그 기준 넘어가면 멈춤 -> cross_state = False, traffic_state = True
                        if int(cross_walk_upline) > 280:        #original:294
                            arduino_device.current_speed = 0
                            time.sleep(0.5)
                            arduino_device.current_speed = -50
                            subcam_device.cross_state = False
                            subcam_device.traffic_state = True
                            break
                        print(cross_walk_upline)
                        #if cross_walk_width * cross_walk_height > 41000:  # 여기에 판단문 만들어주기
                        #    arduino_device.current_speed = 0
                        #    subcam_device.cross_state = False
                        #    subcam_device.traffic_state = True

                    if int(box.cls) == 1 and subcam_device.traffic_state:
                        traffic_size = int(box.xyxy[0][2] - box.xyxy[0][0]) * int(box.xyxy[0][3] - box.xyxy[0][1])
                        traffic_point_y = int(box.xyxy[0][3] + box.xyxy[0][1]) // 2
                        traffic_width = int(box.xyxy[0][2] - box.xyxy[0][0])
                        traffic_r_x = int(box.xyxy[0][0]) + int(traffic_width // 6)
                        traffic_y_x = int(box.xyxy[0][0]) + int(3 * traffic_width // 6)
                        traffic_g_x = int(box.xyxy[0][0]) + int(5 * traffic_width // 6)
                        print(f"R : {[frame[traffic_point_y][traffic_r_x]]}")
                        print(f"Y : {[frame[traffic_point_y][traffic_y_x]]}")
                        print(f"G : {[frame[traffic_point_y][traffic_g_x]]}")
                        color_green = [frame[traffic_point_y][traffic_g_x]]
                        if traffic_size > 10000:
                            print("[find traffic sign]", traffic_size)
                            cv2.imwrite('./traffic_sign.jpg', results[0].plot())
                            time.sleep(0.5)
                            arduino_device.current_speed = 0

                            # green_color = int(
                            #     int(color_green[0][0]) + int(color_green[0][1]) + int(color_green[0][2])) // 3
                            # print(green_color)
                            # if green_color > 250:
                            #     arduino_device.current_speed = 150
                            #     subcam_device.traffic_state = False
                            # else:
                            #     arduino_device.current_speed = 0

                            traffic_hsv = sign_brightness(frame[int(box.xyxy[0][1]):int(box.xyxy[0][3]), int(box.xyxy[0][0]):int(box.xyxy[0][2])])
                            if traffic_hsv[2][2] > 200 and abs(traffic_hsv[2][2] - traffic_hsv[0][2]) > 30 and abs(traffic_hsv[2][2] - traffic_hsv[1][2]) > 30:
                                arduino_device.current_speed = 150
                                subcam_device.traffic_state = False


                    # 혜경_trycode
                    # if subcam_device.avoid_state:
                    #     print("혜경")
                    #     maincam_device.capture = True
                    #     subcam_device.capture = False
                    #     time.sleep(1.4)  # 첫 장애물로 인한 대기 시간
                    #     turn_right()
                    #     time.sleep(1)
                    #     arduino_device.current_speed = 150
                    #     maincam_device.angle_max = int(maincam_device.angle_max * 0.95)
                    #     maincam_device.angle_min = int(maincam_device.angle_min * 0.95)
                    #     subcam_device.capture = True
                    #     subcam_device.avoid_state = False
                    #     subcam_device.traffic_state = False
                    #     subcam_device.cross_state = True
                    #     break
                    # if int(box.cls) == 0 and subcam_device.avoid_state:
                    if (int(box.cls) == 7 or int(box.cls) == 2 or int(box.cls) == 28 or int(box.cls) == 3) and subcam_device.avoid_state: # cls : truck, car, motorcycle
                        car_size = int(box.xyxy[0][2] - box.xyxy[0][0]) * int(box.xyxy[0][3] - box.xyxy[0][1])
                        car_width = int(box.xyxy[0][2] - box.xyxy[0][0])
                        print(f'car_size : {car_size}, car_width : {car_width}')
                        # if int(box.cls) == 2 or int(box.cls) == 3:
                        #     test_size = 6000
                        # elif int(box.cls) == 7 or int(box.cls) == 28:
                        #     test_size = 7000
                        test_size = 130
                        if car_width > test_size:  # originally, 6000, car_width로 해서도 적절한값 찾아보기(대략 70~100 부근 예상)
                            averagex = int((box.xyxy[0][0] + box.xyxy[0][2]) / 2)
                            # 중심점이 왼쪽이면 무시, 가운데쯤 있으면 회피기동
                            print(averagex)
                            if 250 <= averagex <= 400:
                                subcam_device.avoid_state = False
                                car_bias = 350 - averagex
                                turn_right(car_bias=car_bias)
                                arduino_device.current_speed = 150
                                # maincam_device.angle_max = int(maincam_device.angle_max * 0.95)
                                # maincam_device.angle_min = int(maincam_device.angle_min * 0.95)
                                #subcam_device.traffic_state = True
                                time.sleep(3)
                                subcam_device.cross_state = True
                                break

            # if subcam_device.record:
            #     subcam_device.writer_orig.write(frame)
            t2 = time.time_ns()
            # print(f'inference time : {(t2 - t1) / 1e+6}ms')
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
                if maincam_device.line_name == 'llline':
                    conf = 0.6
                else:
                    conf = 0.2
                img_inferenced, drawed_img, line1_ang, line2_ang, mid_bias = inf_angle_mainline(maincam_device.model,
                                                                                                frame,
                                                                                                maincam_device.SAMPLING_RATE,
                                                                                                maincam_device.bev_width_offset,
                                                                                                maincam_device.bev_height_offset,
                                                                                                maincam_device.line_name,
                                                                                                maincam_device.poly_degree,
                                                                                                True,
                                                                                                conf)
                steering_value = 9999.0
                first = line1_ang
                second = line2_ang
                # if maincam_device.line_name == 'llline':
                #     if subcam_device.avoid_state:
                #         first = line1_ang
                #         second = line2_ang
                #     else:
                #         first = line2_ang
                #         second = line1_ang
                if first is not None:
                    idx, steering_value = map_to_steering(first, maincam_device.angle_min, maincam_device.angle_max,
                                                          maincam_device.steering_values)
                    if maincam_device.save_statistics:
                        maincam_device.statistics_list.append([idx, steering_value])
                    if maincam_device.capture:
                        if (mid_bias is not None):#and (not (maincam_device.line_name == 'llline')):
                            arduino_device.current_steering = steering_value + int(mid_bias)
                        else:
                            arduino_device.current_steering = steering_value
                elif second is not None:
                    idx, steering_value = map_to_steering(second, maincam_device.angle_min, maincam_device.angle_max,
                                                          maincam_device.steering_values)
                    if maincam_device.save_statistics:
                        maincam_device.statistics_list.append([idx, steering_value])
                    if maincam_device.capture:
                        if (mid_bias is not None):# and (not (maincam_device.line_name == 'llline')):
                            arduino_device.current_steering = steering_value + int(mid_bias)
                        else:
                            arduino_device.current_steering = steering_value
                if maincam_device.verbose:
                    drawed_img = put_message(drawed_img, 3, [f'rl_ang : {line1_ang}', f'll_ang : {line2_ang}',
                                                             f'steer : {steering_value}'])
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
