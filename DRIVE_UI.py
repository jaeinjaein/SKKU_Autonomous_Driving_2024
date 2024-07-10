import subprocess
import sys
import cv2
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
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
from my_custom_msgs.msg import LidarData, LidarDatas
import multiprocessing
from rplidar import RPLidar, RPLidarException
import math

ard_list = []

class SubCamNode(Node):
    def __init__(self):
        super().__init__('sub_cam_node')
        self.cam_image_publisher = self.create_publisher(Image, 'sub_cam_image', 10)
        self.mtsc_subscriber = self.create_subscription(String, 'msg_mtsc', self.listener_callback_msg_mtsc, 10)
        self.mta_publisher = self.create_publisher(String, 'msg_mta', 10)
        self.cap = None
        self.timer = None
        self.model = YOLO('./models/yolov8x.pt', task='segment')
        self.bridge = CvBridge()
        self.record = False
        self.writer_orig = None
        self.writer_inferenced = None
        self.steering_values = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20]
        self.fps = 5.0
        self.model(np.zeros((360, 640, 3), dtype=np.uint8), conf=0.2)
        
    def start_camera(self, device_index):
        if self.cap is not None:
            self.cap.release()
        self.cap = cv2.VideoCapture(device_index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)

        self.cap.set(cv2.CAP_PROP_FPS, int(self.fps))
        if self.timer is not None:
            self.timer.cancel()
        self.timer = self.create_timer(1.0 / self.fps, self.process_camera_frame)
        if self.record:
            local_time = time.localtime()
            formatted_time = time.strftime('%Y-%m-%d-%H%M%S', local_time)
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self.writer_orig = cv2.VideoWriter(f'./videos/orig/{formatted_time}_sub.mp4', fourcc, self.fps, (640, 360))
            self.writer_inferenced = cv2.VideoWriter(f'./videos/inferenced/{formatted_time}_sub.mp4', fourcc, self.fps, (640, 360))
    
    def process_camera_frame(self):
        if self.cap is None:
            return
        ret, frame = self.cap.read()
        if ret:
            t1 = time.time_ns()
            results = self.model(frame, conf=0.4)
            img_inferenced = results[0].plot()
            msg_inferenced = self.bridge.cv2_to_imgmsg(img_inferenced, 'bgr8')
            self.cam_image_publisher.publish(msg_inferenced)
            for idx, box in enumerate(results[0].boxes):
                if int(box.cls) == 9:
                    traffic_size = int(box.xyxy[0][2] - box.xyxy[0][0]) * int(box.xyxy[0][3] - box.xyxy[0][1])
                    print(traffic_size)
                    if traffic_size > 28000:
                        data = 'speed+000;'
                        msg = String()
                        msg.data = data
                        self.mta_publisher.publish(msg)
                    
            if self.record:
                self.writer_orig.write(frame)
            t2 = time.time_ns()
            print(f'inference time : {(t2 - t1) / 1e+6}ms')

    def start_video(self, video_path):
        if self.cap is not None:
            self.cap.release()
        self.cap = cv2.VideoCapture(video_path)
        if self.timer is not None:
            self.timer.cancel()
        self.timer = self.create_timer(1.0 / self.fps, self.process_camera_frame)
        if self.record:
            local_time = time.localtime()
            formatted_time = time.strftime('%Y-%m-%d-%H%M%S', local_time)
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self.writer_orig = cv2.VideoWriter(f'./videos/orig/{formatted_time}.mp4', fourcc, self.fps, (640, 360))
            self.writer_inferenced = cv2.VideoWriter(f'./videos/inferenced/{formatted_time}.mp4', fourcc, self.fps, (640, 360))

    def destroy_node(self):
        super().destroy_node()
        if self.cap is not None:
            self.cap.release()
            
    def stop_camera(self):
        if self.cap is not None:
            self.cap.release()
        if self.timer is not None:
            self.timer.cancel()
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
    
    def listener_callback_msg_mtsc(self, msg):
        exec(msg.data)
        

class CamNode(Node):
    def __init__(self):
        super().__init__('cam_node')
        self.mta_publisher = self.create_publisher(String, 'msg_mta', 10)
        self.mtc_subscriber = self.create_subscription(String, 'msg_mtc', self.listener_callback_msg_mtc, 10)
        self.cam_image_publisher = self.create_publisher(Image, 'cam_image', 10)
        self.cam_image_infer_publisher = self.create_publisher(Image, 'cam_image_infer', 10)
        self.cap = None
        self.timer = None
        self.bridge = CvBridge()
        self.model = YOLO('./last.engine', task='segment')
        self.record = False
        self.writer_orig = None
        self.writer_inferenced = None
        self.statistics_list = []
        self.save_statistics = True
        self.SAMPLING_RATE = 0.8
        self.bev_height_offset = 0.2
        self.bev_width_offset = 0.27
        self.angle_min = -80
        self.angle_max = 80
        self.steering_values = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20]
        self.poly_degree = 2
        self.fps = 10.0
        self.verbose = True
        self.load_params()
        self.model(np.zeros((360, 640, 3), dtype=np.uint8), conf=0.2)
        #self.start_camera(0)

    def listener_callback_msg_mtc(self, msg):
        exec(msg.data)

    def start_camera(self, device_index):
        if self.cap is not None:
            self.cap.release()
        self.cap = cv2.VideoCapture(device_index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)
        self.cap.set(cv2.CAP_PROP_FPS, int(self.fps))
        if self.timer is not None:
            self.timer.cancel()
        self.statistics_list = []
        self.timer = self.create_timer(1.0 / self.fps, self.publish_camera_frame)
        if self.record:
            local_time = time.localtime()
            formatted_time = time.strftime('%Y-%m-%d-%H%M%S', local_time)
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self.writer_orig = cv2.VideoWriter(f'./videos/orig/{formatted_time}.mp4', fourcc, self.fps, (640, 360))
            self.writer_inferenced = cv2.VideoWriter(f'./videos/inferenced/{formatted_time}.mp4', fourcc, self.fps, (640, 360))

    def publish_camera_frame(self):
        if self.cap is None:
            return
        ret, frame = self.cap.read()
        if ret:
            try:
                t1 = time.time_ns()
                img_inferenced, drawed_img, line1_ang, line2_ang, mid_bias = inf_angle_mainline(self.model, frame, self.SAMPLING_RATE, self.bev_width_offset, self.bev_height_offset, self.poly_degree)
                
                steering_value = -9999.0
                if line1_ang != None:
                    idx, steering_value = map_to_steering(line1_ang, self.angle_min, self.angle_max, self.steering_values)
                    if self.save_statistics:
                        self.statistics_list.append([idx, steering_value])
                    if mid_bias != None:
                        self.publish_angle(steering_value + int(mid_bias))
                    else:
                        self.publish_angle(steering_value)
                elif line2_ang != None:
                    idx, steering_value = map_to_steering(line2_ang)
                    if self.save_statistics:
                        self.statistics_list.append([idx, steering_value])
                    if mid_bias != None:
                        self.publish_angle(steering_value + int(mid_bias))
                    else:
                        self.publish_angle(steering_value)
                if self.verbose:
                    drawed_img = put_message(drawed_img, 3, [f'rl_ang : {line1_ang}',f'll_ang : {line2_ang}', f'steer : {steering_value}'])
                
                if self.record:
                    self.writer_orig.write(frame)
                    self.writer_inferenced.write(drawed_img)
                t3 = time.time_ns()
                msg_inferenced = self.bridge.cv2_to_imgmsg(img_inferenced, 'bgr8')
                self.cam_image_publisher.publish(msg_inferenced)
                msg_drawed = self.bridge.cv2_to_imgmsg(drawed_img, 'bgr8')
                self.cam_image_infer_publisher.publish(msg_drawed)
                t4 = time.time_ns()
                print(f"topic-publish delay : {(t4 - t3) / 1e+6}ms")
                t2 = time.time_ns()
                print(f"inference time : {(t2 - t1) / 1000000}ms")
                if mid_bias != None:
                    print(f'mid_bias : {mid_bias}')
            except Exception as e:
                print(f"An error occurred: {e}")
                traceback.print_exc()
                
    def publish_angle(self, value):
        data = 'angle%02d;' % value
        msg = String()
        msg.data = data
        self.mta_publisher.publish(msg)

    def start_video(self, video_path):
        if self.cap is not None:
            self.cap.release()
        self.cap = cv2.VideoCapture(video_path)
        if self.timer is not None:
            self.timer.cancel()
        if self.save_statistics and len(self.statistics_list) > 0:
            local_time = time.localtime()
            formatted_time = time.strftime('%Y-%m-%d-%H%M%S', local_time)
            save_array = np.array(self.statistics_list)
            np.save(f'./log/steering_log/{formatted_time}_steering', save_array)
        self.statistics_list = []
        self.timer = self.create_timer(1.0 / self.fps, self.publish_camera_frame)
        if self.record:
            local_time = time.localtime()
            formatted_time = time.strftime('%Y-%m-%d-%H%M%S', local_time)
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self.writer_orig = cv2.VideoWriter(f'./videos/orig/{formatted_time}.mp4', fourcc, self.fps, (640, 360))
            self.writer_inferenced = cv2.VideoWriter(f'./videos/inferenced/{formatted_time}.mp4', fourcc, self.fps, (640, 360))

    def destroy_node(self):
        super().destroy_node()
        if self.cap is not None:
            self.cap.release()
            
    def stop_camera(self):
        if self.save_statistics and len(self.statistics_list) > 0:
            local_time = time.localtime()
            formatted_time = time.strftime('%Y-%m-%d-%H%M%S', local_time)
            save_array = np.array(self.statistics_list)
            np.save(f'./log/steering_log/{formatted_time}_steering', save_array)
        if self.cap is not None:
            self.cap.release()
        if self.timer is not None:
            self.timer.cancel()
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

class LidarNode(Node):
    def __init__(self):
        super().__init__('lidar_node')
        self.mtl_subscriber = self.create_subscription(String, 'msg_mtl', self.listener_callback_msg_mtl, 10)
        self.lidar_image_publisher = self.create_publisher(Image, 'lidar_image', 10)
        self.timer = None
        self.window_width = 360
        self.window_height = 360
        self.max_distance = 6000
        self.scale = self.window_width / (2 * self.max_distance)
        self.lidar_session = None
        self.bridge = CvBridge()
        self.lidar_thread = threading.Thread(target=self.lidar_main, daemon=True)
        self.lidar_thread.start()

    def start_lidar(self, port):
        lidar_session_cp = RPLidar(port)
        info = lidar_session_cp.get_info()
        print(info)
        health = lidar_session_cp.get_health()
        print(health)
        lidar_session_cp.start_motor()
        time.sleep(2)
        self.lidar_session = lidar_session_cp
        
    def lidar_main(self):
        while True:
            if self.lidar_session is None:
                continue
            try:
                for scan in self.lidar_session.iter_scans():
                    image = np.zeros((self.window_height, self.window_width, 3), dtype=np.uint8)
                    self.draw_lidar_scan(image, scan)
                    # publish the image
                    msg_img = self.bridge.cv2_to_imgmsg(image, 'bgr8')
                    self.lidar_image_publisher.publish(msg_img)
                    # and process the datas if you need
            except:
                pass
            time.sleep(0.01)
                
    def stop_lidar(self):
        self.lidar_session.stop()
        self.lidar_session.stop_motor()
        self.lidar_session.disconnect()
        self.lidar_session = None
        
    def polar_to_cartesian(self, angle, distance):
        """폴라 좌표를 카르테시안 좌표로 변환"""
        rads = math.radians(angle)
        x = distance * math.cos(rads)
        y = distance * math.sin(rads)
        return x, y
        
    def draw_lidar_scan(self, image, scan):
        """라이다 스캔 데이터를 화면에 그리기"""
        for (_, angle, distance) in scan:
            if distance > self.max_distance:
                continue
            x, y = self.polar_to_cartesian(angle, distance)
            x = int(self.window_width / 2 + x * self.scale)
            y = int(self.window_height / 2 - y * self.scale)
            cv2.circle(image, (x, y), 2, (0, 255, 0), -1)
            
    def listener_callback_msg_mtl(self, msg):
        exec(msg.data)

class ArdNode(Node):
    def __init__(self):
        super().__init__('ard_node')
        self.subscriber_mta = self.create_subscription(
            String,
            'msg_mta',
            self.listener_mta,
            10)
        self.publisher = self.create_publisher(String, 'msg_atm', 10)
        self.timer = None
        self.port = ''
        self.serial_session = None

    def start_arduino(self, port):
        self.port = port
        self.serial_session = serial.Serial(port=self.port, baudrate=9600)
        if self.timer is not None:
            self.timer.cancel()
        self.timer = self.create_timer(1.0 / 1.0, self.publish_arduino_data)

    def publish_arduino_data(self):
        pass
        
    def stop_arduino(self):
        self.serial_session.close()
        self.port = ''
        self.serial_session = None
        self.timer.cancel()
        self.timer = None
    
    def listener_mta(self, data):
        if data.data.startswith('exec'):
            exec(data.data[5:])
        else:
            if self.port != '':
                self.serial_session.write(data.data.encode())

class MainNode(Node):
    def __init__(self):
        super().__init__('main_node')
        
        # topics between camera and main
        self.sub_cam_image = self.create_subscription(Image, 'cam_image', self.listener_callback_cam_image, 10)
        self.sub_cam_image_infer = self.create_subscription(Image, 'cam_image_infer', self.listener_callback_cam_image_infer, 10)
        self.sub_msg_ctm = self.create_subscription(String, 'msg_ctm', self.listener_callback_msg_ctm, 10)
        self.mtc_publisher = self.create_publisher(String, 'msg_mtc', 10)
        
        # topics between sub camera and main
        self.sub_scam_image = self.create_subscription(Image, 'sub_cam_image', self.listener_callback_sub_cam_image, 10)
        self.sub_msg_sctm = self.create_subscription(String, 'msg_sctm', self.listener_callback_msg_sctm, 10)
        self.mtsc_publisher = self.create_publisher(String, 'msg_mtsc', 10)
        
        # topics between lidar and main
        self.sub_lidar_image = self.create_subscription(Image, 'lidar_image', self.listener_callback_lidar_image, 10)
        self.sub_msg_ltm = self.create_subscription(String, 'msg_ltm', self.listener_callback_msg_ltm, 10)
        self.mtl_publisher = self.create_publisher(String, 'msg_mtl', 10)
        
        # topics between ard and main
        self.sub_msg_atm = self.create_subscription(String, 'msg_atm', self.listener_callback_msg_atm, 10)
        self.mta_publisher = self.create_publisher(String, 'msg_mta', 10)

        self.bridge = CvBridge()
        self.cv_image = np.zeros((720, 1280, 3), dtype=np.uint8)
        
    def listener_callback_cam_image(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        h, w = image.shape[:2]
        if h != 360 or w != 640:
            image = cv2.resize(image, (640, 360))
        self.cv_image[:360, :640] = image  # left high image
    
    def listener_callback_cam_image_infer(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        h, w = image.shape[:2]
        if h != 360 or w != 640:
            image = cv2.resize(image, (640, 360))
        self.cv_image[:360, 640:] = image  # right high image
    
    def listener_callback_sub_cam_image(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        h, w = image.shape[:2]
        if h != 360 or w != 640:
            image = cv2.resize(image, (640, 360))
        self.cv_image[360:, :640] = image  # left below image
        
    def listener_callback_lidar_image(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.cv_image[360:, 640:1000] = image  # right below image
        
    def listener_callback_msg_ctm(self, msg):
        # for msg camera to main [msg : String --> String.data]
        pass
        
    def listener_callback_msg_sctm(self, msg):
        # for msg sub camera to main [msg : String --> String.data]
        pass
        
    def listener_callback_msg_ltm(self, msg):
        # for msg lidar to main [msg : String --> String.data]
        pass
        
    def listener_callback_msg_atm(self, msg):
        # for msg arduino to main [msg : String --> String.data]
        pass

    # message functions for main node to arduino node
    def publish_setting(self):
        data = 'setting;'
        msg = String()
        msg.data = data
        self.mta_publisher.publish(msg)
        
    def publish_speed(self, value):
        data = 'speed'
        if value >= 0:
            data += '+'
        else:
            data += '-'
        data += '%03d;' % self.abs_value(value)
        msg = String()
        msg.data = data
        self.mta_publisher.publish(msg)
        
    def publish_angle(self, value):
        data = 'angle%02d;' % value
        msg = String()
        msg.data = data
        self.mta_publisher.publish(msg)
    
    def abs_value(self, value):
        if value >= 0:
            return value
        else:
            return -1 * value
            
    def publish_mta_rawdata(self, data):
        msg = String()
        msg.data = data
        self.mta_publisher.publish(msg)
            
    # message functions for main node to cam node
    def publish_mtc_rawdata(self, data):
        msg = String()
        msg.data = data
        self.mtc_publisher.publish(msg)
        
    def publish_mtl_rawdata(self, data):
        msg = String()
        msg.data = data
        self.mtl_publisher.publish(msg)
        
    def publish_mtsc_rawdata(self, data):
        msg = String()
        msg.data = data
        self.mtsc_publisher.publish(msg)
        
class MyApp(QWidget):
    def __init__(self, main_node):
        super().__init__()
        self.main_node = main_node
        self.cam_conn, self.sub_cam_conn, self.lidar_conn, self.ard_conn = False, False, False, False
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
        self.main_node.publish_mtc_rawdata(f"self.record = {self.checkboxRecord.isChecked()}")

    def selectCamDevice(self, comboBox, selectButton):
        if not self.cam_conn:
            index = int(comboBox.currentText())
            comboBox.setEnabled(False)
            self.main_node.publish_mtc_rawdata(f"self.start_camera({index})")
            selectButton.setText("Disconnect")
            self.cam_conn = True
            self.checkboxRecord.setEnabled(False)
        else:
            comboBox.setEnabled(True)
            self.main_node.publish_mtc_rawdata(f"self.stop_camera()")
            selectButton.setText("Connect")
            self.cam_conn = False
            self.checkboxRecord.setEnabled(True)
            
    def selectSubCamDevice(self, comboBox, selectButton):
        if not self.sub_cam_conn:
            index = int(comboBox.currentText())
            comboBox.setEnabled(False)
            self.main_node.publish_mtsc_rawdata(f"self.start_camera({index})")
            selectButton.setText("Disconnect")
            self.sub_cam_conn = True
            self.checkboxRecord.setEnabled(False)
        else:
            comboBox.setEnabled(True)
            self.main_node.publish_mtsc_rawdata(f"self.stop_camera()")
            selectButton.setText("Connect")
            self.sub_cam_conn = False
            self.checkboxRecord.setEnabled(False)

    def selectLidarDevice(self, comboBox, selectButton):
        if not self.lidar_conn:
            port = comboBox.currentText()
            comboBox.setEnabled(False)
            selectButton.setText("Disconnect")
            self.main_node.publish_mtl_rawdata(f"self.start_lidar('{port}')")
            self.lidar_conn = True
        else:
            comboBox.setEnabled(True)
            self.main_node.publish_mtl_rawdata(f"self.stop_lidar()")
            selectButton.setText("Connect")
            self.lidar_conn = False

    def selectArdDevice(self, comboBox, selectButton):
        if not self.ard_conn:
            port = comboBox.currentText()
            comboBox.setEnabled(False)
            selectButton.setText("Disconnect")
            self.main_node.publish_mta_rawdata(f"exec self.start_arduino('{port}')")
            self.ard_conn = True
        else:
            comboBox.setEnabled(True)
            self.main_node.publish_mta_rawdata(f"exec self.stop_arduino()")
            selectButton.setText("Connect")
            self.ard_conn = False

    def TestStartButtonClicked(self):
        if self.checkboxTest.isChecked():
            if self.cam_conn:
                return
            video_path = './videos/orig/2024-07-03-060729.mp4'
            self.main_node.publish_mtc_rawdata(f"self.start_video('{video_path}')")
            
    def updateImage(self):
        if self.main_node.cv_image is not None:
            self.showImage(self.main_node.cv_image)

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
        self.main_node.publish_speed(255)
        
    def speed150ButtonClicked(self):
        self.main_node.publish_speed(150)
        
    def speed080ButtonClicked(self):
        self.main_node.publish_speed(80)
        
    def speed000ButtonClicked(self):
        self.main_node.publish_speed(0)
    
    def speedm150ButtonClicked(self):
        self.main_node.publish_speed(-150)
        
    def settingAngleButtonClicked(self):
        self.main_node.publish_setting()
        
    def deviceReloadButtonClicked(self):
        commands = [
            "echo 'dlwodls8747' | sudo -S chmod 777 /dev/ttyUSB*",
            "echo 'dlwodls8747' | sudo -S chmod 777 /dev/ttyACM*"
        ]
        # 각 명령어 실행
        for command in commands:
            result = subprocess.run(command, shell=True, capture_output=True, text=True)
            print(result.stdout)
            if result.returncode != 0:
                print(f"Error: {result.stderr}")
        self.populateCamComboBox(self.camComboBox)
        self.populateSerialComboBox(self.ardComboBox)
        self.populateSerialComboBox(self.lidarComboBox)
    
    def updateCamParamButtonClicked(self):
        self.main_node.publish_mtc_rawdata("self.load_params()")
        self.main_node.publish_mtsc_rawdata("self.load_params()")
    
    
def cam_node_process():
    rclpy.init(args=None)
    cam_node = CamNode()
    rclpy.spin(cam_node)
    cam_node.destroy_node()
    rclpy.shutdown()
    
def ard_node_process():
    rclpy.init(args=None)
    ard_node = ArdNode()
    rclpy.spin(ard_node)
    ard_node.destroy_node()
    rclpy.shutdown()
    
def lidar_node_process():
    rclpy.init(args=None)
    lidar_node = LidarNode()
    rclpy.spin(lidar_node)
    lidar_node.destroy_node()
    rclpy.shutdown()
    
def sub_cam_node_process():
    rclpy.init(args=None)
    sub_cam_node = SubCamNode()
    rclpy.spin(sub_cam_node)
    sub_cam_node.destroy_node()
    rclpy.shutdown()


def main(args=None):
    ctx = multiprocessing.get_context('spawn')
    process_cam = multiprocessing.Process(target=cam_node_process, daemon=True)
    process_sub_cam = multiprocessing.Process(target=sub_cam_node_process, daemon=True)
    process_lidar = multiprocessing.Process(target=lidar_node_process, daemon=True)
    process_ard = multiprocessing.Process(target=ard_node_process, daemon=True)
    process_cam.start()
    process_sub_cam.start()
    process_lidar.start()
    process_ard.start()
    
    rclpy.init(args=args)
    main_node = MainNode()

    executor = SingleThreadedExecutor()
    executor.add_node(main_node)
    

    app = QApplication(sys.argv)
    ex = MyApp(main_node)
    
    # Spin ROS node in a separate thread to avoid blocking the main thread
    thread_main = threading.Thread(target=executor.spin, daemon=True)
    thread_main.start()
    
    
    sys.exit(app.exec_())
    
    thread_main.join()
    process_cam.join()
    process_sub_cam.join()
    process_lidar.join()
    process_ard.join()

if __name__ == '__main__':
    main()
