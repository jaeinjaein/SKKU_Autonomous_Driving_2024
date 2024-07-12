import subprocess
import sys
import cv2
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QGroupBox, QCheckBox, QLabel, QComboBox, QPushButton, QGridLayout
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import QTimer, Qt
import threading
import numpy as np
import serial.tools.list_ports
from inference import inference_image, RunningAverage, inf_angle_mainline
from ultralytics import YOLO
import time
import serial
from util.tools import map_to_n_levels, put_message, map_to_steering
import traceback
from copy import deepcopy
import multiprocessing
from rplidar import RPLidar, RPLidarException
import math

ard_list = []


class lidar():
    def __init__(self):
        pass

class subcam():
    def __init__(self):
        self.cap = None
        self.model = YOLO('./models/yolov8x.pt', task='detect')
        self.model.to('mps')
        self.record = False
        self.writer_orig = None
        self.writer_inferenced = None
        self.update_img = np.zeros((360, 640, 3), dtype=np.uint8)
        self.steering_values = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20]
        self.fps = 5.0
        self.capture = False
        self.model(np.zeros((360, 640, 3), dtype=np.uint8), conf=0.2)
        
    def start_camera(self, device_index):
        if self.cap is not None:
            self.cap.release()
        self.cap = cv2.VideoCapture(device_index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)
        self.cap.set(cv2.CAP_PROP_FPS, int(self.fps))
        if self.record:
            local_time = time.localtime()
            formatted_time = time.strftime('%Y-%m-%d-%H%M%S', local_time)
            fourcc = cv2.VideoWriter_fourcc(*'H264')
            self.writer_orig = cv2.VideoWriter(f'./videos/orig/{formatted_time}_sub.mp4', fourcc, self.fps, (640, 360))
            self.writer_inferenced = cv2.VideoWriter(f'./videos/inferenced/{formatted_time}_sub.mp4', fourcc, self.fps, (640, 360))
        self.capture = True

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
        self.model = YOLO('./models/yolov8s-ep200-unf-d3.pt', task='segment')
        self.model.to('mps')
        self.model.half()
        self.record = False
        self.writer_orig = None
        self.writer_inferenced = None
        self.statistics_list = []
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

    def start_camera(self, device_index):
        if self.cap is not None:
            self.cap.release()
        self.cap = cv2.VideoCapture(device_index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)
        self.cap.set(cv2.CAP_PROP_FPS, int(self.fps))
        self.statistics_list = []
        if self.record:
            local_time = time.localtime()
            formatted_time = time.strftime('%Y-%m-%d-%H%M%S', local_time)
            fourcc = cv2.VideoWriter_fourcc(*'H264')
            self.writer_orig = cv2.VideoWriter(f'./videos/orig/{formatted_time}.mp4', fourcc, self.fps, (640, 360))
            self.writer_inferenced = cv2.VideoWriter(f'./videos/inferenced/{formatted_time}.mp4', fourcc, self.fps, (640, 360))
        self.capture = True

    def start_video(self, video_path):
        if self.cap is not None:
            self.cap.release()
        self.cap = cv2.VideoCapture(video_path)
        if self.save_statistics and len(self.statistics_list) > 0:
            local_time = time.localtime()
            formatted_time = time.strftime('%Y-%m-%d-%H%M%S', local_time)
            save_array = np.array(self.statistics_list)
            np.save(f'./log/steering_log/{formatted_time}_steering', save_array)
        self.statistics_list = []
        if self.record:
            local_time = time.localtime()
            formatted_time = time.strftime('%Y-%m-%d-%H%M%S', local_time)
            fourcc = cv2.VideoWriter_fourcc(*'H264')
            self.writer_orig = cv2.VideoWriter(f'./videos/orig/{formatted_time}.mp4', fourcc, self.fps, (640, 360))
            self.writer_inferenced = cv2.VideoWriter(f'./videos/inferenced/{formatted_time}.mp4', fourcc, self.fps, (640, 360))
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

# class LidarNode(Node):
#     def __init__(self):
#         super().__init__('lidar_node')
#         self.mtl_subscriber = self.create_subscription(String, 'msg_mtl', self.listener_callback_msg_mtl, 10)
#         self.lidar_image_publisher = self.create_publisher(Image, 'lidar_image', 10)
#         self.timer = None
#         self.window_width = 360
#         self.window_height = 360
#         self.max_distance = 6000
#         self.scale = self.window_width / (2 * self.max_distance)
#         self.lidar_session = None
#         self.bridge = CvBridge()
#         self.lidar_thread = threading.Thread(target=self.lidar_main, daemon=True)
#         self.lidar_thread.start()
#
#     def start_lidar(self, port):
#         lidar_session_cp = RPLidar(port)
#         info = lidar_session_cp.get_info()
#         print(info)
#         health = lidar_session_cp.get_health()
#         print(health)
#         lidar_session_cp.start_motor()
#         time.sleep(2)
#         self.lidar_session = lidar_session_cp
#
#     def lidar_main(self):
#         while True:
#             if self.lidar_session is None:
#                 continue
#             try:
#                 for scan in self.lidar_session.iter_scans():
#                     image = np.zeros((self.window_height, self.window_width, 3), dtype=np.uint8)
#                     self.draw_lidar_scan(image, scan)
#                     # publish the image
#                     msg_img = self.bridge.cv2_to_imgmsg(image, 'bgr8')
#                     self.lidar_image_publisher.publish(msg_img)
#                     # and process the datas if you need
#             except:
#                 pass
#             time.sleep(0.01)
#
#     def stop_lidar(self):
#         self.lidar_session.stop()
#         self.lidar_session.stop_motor()
#         self.lidar_session.disconnect()
#         self.lidar_session = None
#
#     def polar_to_cartesian(self, angle, distance):
#         """폴라 좌표를 카르테시안 좌표로 변환"""
#         rads = math.radians(angle)
#         x = distance * math.cos(rads)
#         y = distance * math.sin(rads)
#         return x, y
#
#     def draw_lidar_scan(self, image, scan):
#         """라이다 스캔 데이터를 화면에 그리기"""
#         for (_, angle, distance) in scan:
#             if distance > self.max_distance:
#                 continue
#             x, y = self.polar_to_cartesian(angle, distance)
#             x = int(self.window_width / 2 + x * self.scale)
#             y = int(self.window_height / 2 - y * self.scale)
#             cv2.circle(image, (x, y), 2, (0, 255, 0), -1)
#
#     def listener_callback_msg_mtl(self, msg):
#         exec(msg.data)

class arduino():
    def __init__(self):
        self.port = ''
        self.serial_session = None

    def start_arduino(self, port):
        self.port = port
        self.serial_session = serial.Serial(port=self.port, baudrate=9600)

    # def send_message(self, data):
    #     if self.serial_session != None:
    #         try:
    #             if self.serial_session.isOpen():
    #                 self.serial_session.write(data.encode())
    #                 self.serial_session.

    def send_angle(self, value):
        if self.serial_session != None:
            self.serial_session.flush()
            data = 'angle%02d;' % value
            print(self.serial_session.write(data.encode()))

    def send_speed(self, value):
        if self.serial_session != None:
            self.serial_session.flush()
            data = 'speed'
            if value >= 0:
                data += '+'
            else:
                data += '-'
            data += '%03d;' % self.abs_value(value)
            self.serial_session.write(data.encode())

    def abs_value(self, value):
        if value >= 0:
            return value
        else:
            return -1 * value

    def stop_arduino(self):
        self.serial_session.close()
        self.port = ''
        self.serial_session = None

    def setting(self):
        if self.serial_session != None:
            self.serial_session.write('setting;'.encode())
        
class MyApp(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):
        layout = QGridLayout()

        # Left top checkbox group
        self.leftTopGroup = QGroupBox('Modes')
        self.checkbox1 = QCheckBox('DRIVE')
        self.checkbox2 = QCheckBox('AV/TR')
        self.checkbox3 = QCheckBox('PARK')
        self.checkboxTest = QCheckBox('TEST')
        self.checkboxRecord = QCheckBox('RECORD')
        self.checkbox1.stateChanged.connect(lambda: self.modeChanged(self.checkbox1))
        self.checkbox2.stateChanged.connect(lambda: self.modeChanged(self.checkbox2))
        self.checkbox3.stateChanged.connect(lambda: self.modeChanged(self.checkbox3))
        self.checkboxTest.stateChanged.connect(lambda: self.modeChanged(self.checkboxTest))
        self.checkboxRecord.stateChanged.connect(self.recordChanged)
        leftTopLayout = QVBoxLayout()
        leftTopLayout.addWidget(self.checkbox1)
        leftTopLayout.addWidget(self.checkbox2)
        leftTopLayout.addWidget(self.checkbox3)
        leftTopLayout.addWidget(self.checkboxTest)
        leftTopLayout.addWidget(self.checkboxRecord)
        self.leftTopGroup.setLayout(leftTopLayout)
        
        # Right top driver select group
        self.rightTopGroup = QGroupBox('Driver Select')
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
        self.TestStartButton = QPushButton('Test Start')
        self.speed255Button = QPushButton('Speed : 255')
        self.speed150Button = QPushButton('Speed : 150')
        self.speed080Button = QPushButton('Speed : 80')
        self.speed000Button = QPushButton('Speed : 0')
        self.speedm150Button = QPushButton('Speed : -150')
        self.settingAngleButton = QPushButton('Car Angle Setting')
        self.updateCamParamButton = QPushButton('Update CAM Parameters')
        self.TestStartButton.clicked.connect(self.TestStartButtonClicked)
        self.speed255Button.clicked.connect(self.speed255ButtonClicked)
        self.speed150Button.clicked.connect(self.speed150ButtonClicked)
        self.speed080Button.clicked.connect(self.speed080ButtonClicked)
        self.speed000Button.clicked.connect(self.speed000ButtonClicked)
        self.speedm150Button.clicked.connect(self.speedm150ButtonClicked)
        self.settingAngleButton.clicked.connect(self.settingAngleButtonClicked)
        self.updateCamParamButton.clicked.connect(self.updateCamParamButtonClicked)
        rightBottomLayout = QVBoxLayout()
        rightBottomLayout.addWidget(self.TestStartButton)
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
        self.setWindowTitle('PyQT Layout Example')
        self.setGeometry(300, 300, 800, 600)
        
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.updateImage)
        self.timer.start(30)  # Update the image every 100 ms
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
            if checkbox == self.checkbox1:
                self.checkbox2.setChecked(False)
                self.checkbox3.setChecked(False)
                self.checkboxTest.setChecked(False)
            elif checkbox == self.checkbox2:
                self.checkbox1.setChecked(False)
                self.checkbox3.setChecked(False)
                self.checkboxTest.setChecked(False)
            elif checkbox == self.checkbox3:
                self.checkbox1.setChecked(False)
                self.checkbox2.setChecked(False)
                self.checkboxTest.setChecked(False)
            elif checkbox == self.checkboxTest:
                self.checkbox1.setChecked(False)
                self.checkbox2.setChecked(False)
                self.checkbox3.setChecked(False)
    
    def recordChanged(self):
        maincam_device.record = self.checkboxRecord.isChecked()
        subcam_device.record = self.checkboxRecord.isChecked()

    def selectCamDevice(self, comboBox, selectButton):
        if maincam_device.cap is None:
            index = int(comboBox.currentText())
            comboBox.setEnabled(False)
            maincam_device.start_camera(index)
            selectButton.setText("Disconnect")
            self.checkboxRecord.setEnabled(False)
        else:
            comboBox.setEnabled(True)
            maincam_device.stop_camera()
            selectButton.setText("Connect")
            self.checkboxRecord.setEnabled(True)
            
    def selectSubCamDevice(self, comboBox, selectButton):
        if subcam_device.cap is None:
            index = int(comboBox.currentText())
            comboBox.setEnabled(False)
            subcam_device.start_camera(index)
            selectButton.setText("Disconnect")
            self.checkboxRecord.setEnabled(False)
        else:
            comboBox.setEnabled(True)
            subcam_device.stop_camera()
            selectButton.setText("Connect")
            self.checkboxRecord.setEnabled(False)

    def selectLidarDevice(self, comboBox, selectButton):
        if not self.lidar_conn:
            port = comboBox.currentText()
            comboBox.setEnabled(False)
            selectButton.setText("Disconnect")
            #self.main_node.publish_mtl_rawdata(f"self.start_lidar('{port}')")
            self.lidar_conn = True
        else:
            comboBox.setEnabled(True)
            #self.main_node.publish_mtl_rawdata(f"self.stop_lidar()")
            selectButton.setText("Connect")
            self.lidar_conn = False

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

    def TestStartButtonClicked(self):
        if self.checkboxTest.isChecked():
            if maincam_device.cap != None:
                return
            video_path = './test_video.mp4'
            maincam_device.start_video(video_path)
            if subcam_device.cap != None:
                return
            video_path = './test_video_sub.mp4'
            subcam_device.start_video(video_path)
            
    def updateImage(self):
        main_image = np.zeros((720, 1280, 3), np.uint8)
        main_image[:360, :1280] = maincam_device.update_img

        main_image[360:, :640] = subcam_device.update_img
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
        arduino_device.send_speed(255)
        
    def speed150ButtonClicked(self):
        arduino_device.send_speed(150)
        
    def speed080ButtonClicked(self):
        arduino_device.send_speed(80)
        
    def speed000ButtonClicked(self):
        arduino_device.send_speed(0)
    
    def speedm150ButtonClicked(self):
        arduino_device.send_speed(-150)
        
    def settingAngleButtonClicked(self):
        arduino_device.setting()
        
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

def subcam_task():
    if subcam_device.cap != None and subcam_device.capture:
        ret, frame = subcam_device.cap.read()
        if ret:
            t1 = time.time_ns()
            results = subcam_device.model(frame, device='mps', conf=0.4)
            subcam_device.update_img = results[0].plot()
            for idx, box in enumerate(results[0].boxes):
                if int(box.cls) == 9:
                    traffic_size = int(box.xyxy[0][2] - box.xyxy[0][0]) * int(box.xyxy[0][3] - box.xyxy[0][1])
                    if traffic_size > 28000:
                        data = 'speed+000;'
                        # data를 ard_device에 전하기
            if subcam_device.record:
                #rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                subcam_device.writer_orig.write(frame)
            t2 = time.time_ns()
            #print(f'inference time : {(t2 - t1) / 1e+6}ms')
            if subcam_device.update_img.shape[0] != 360 or subcam_device.update_img.shape[1] != 640:
                subcam_device.update_img = cv2.resize(subcam_device.update_img, (640, 360))

    global subcam_timer
    subcam_timer = threading.Timer(1 / subcam_device.fps, subcam_task)
    subcam_timer.start()


def maincam_task():
    if maincam_device.cap != None and maincam_device.capture:
        ret, frame = maincam_device.cap.read()
        if ret:
            try:
                t1 = time.time_ns()
                img_inferenced, drawed_img, line1_ang, line2_ang, mid_bias = inf_angle_mainline(maincam_device.model, frame, maincam_device.SAMPLING_RATE, maincam_device.bev_width_offset, maincam_device.bev_height_offset, maincam_device.poly_degree)
                steering_value = 9999.0
                if line1_ang != None:
                    idx, steering_value = map_to_steering(line1_ang, maincam_device.angle_min, maincam_device.angle_max, maincam_device.steering_values)
                    if maincam_device.save_statistics:
                        maincam_device.statistics_list.append([idx, steering_value])
                    if mid_bias != None:
                        arduino_device.send_angle(steering_value + int(mid_bias))
                        print(f'mid_bias : {mid_bias}')
                    else:
                        arduino_device.send_angle(steering_value)
                elif line2_ang != None:
                    idx, steering_value = map_to_steering(line2_ang, maincam_device.angle_min, maincam_device.angle_max, maincam_device.steering_values)
                    if maincam_device.save_statistics:
                        maincam_device.statistics_list.append([idx, steering_value])
                    if mid_bias != None:
                        arduino_device.send_angle(steering_value + int(mid_bias))
                        print(f'mid_bias : {mid_bias}')
                    else:
                        arduino_device.send_angle(steering_value)
                if maincam_device.verbose:
                    drawed_img = put_message(drawed_img, 3, [f'rl_ang : {line1_ang}', f'll_ang : {line2_ang}', f'steer : {steering_value}'])
                if maincam_device.record:
                    #rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    maincam_device.writer_orig.write(frame)
                    maincam_device.writer_inferenced.write(drawed_img)
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
    maincam_timer = threading.Timer(1 / maincam_device.fps, maincam_task)
    maincam_timer.start()


def lidar_task():
    pass



subcam_device = subcam()
subcam_device.load_params()
subcam_timer = threading.Timer(1 / subcam_device.fps, subcam_task)

maincam_device = maincam()
maincam_device.load_params()
maincam_timer = threading.Timer(1 / maincam_device.fps, maincam_task)

lidar_device = lidar()
lidar_timer = threading.Timer(1.0, lidar_task)

arduino_device = arduino()

subcam_timer.start()
maincam_timer.start()

app = QApplication(sys.argv)
ex = MyApp()

sys.exit(app.exec_())
subcam_timer.cancel()
maincam_timer.cancel()