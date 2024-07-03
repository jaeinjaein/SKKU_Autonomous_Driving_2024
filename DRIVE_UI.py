import subprocess
import sys
import cv2
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QGroupBox, QCheckBox, QLabel, QComboBox, QPushButton, QGridLayout
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import QTimer, Qt
import threading
import numpy as np
import serial.tools.list_ports
from inference import inference_image, RunningAverage, inf_angle, inf_angle_mainline
from ultralytics import YOLO
import time
import serial
from util.tools import map_to_n_levels, put_message

ard_list = []

class CamNode(Node):
    def __init__(self):
        super().__init__('cam_node')
     
        self.publisher = self.create_publisher(Image, 'cam_img', 10)
        self.pta_publisher = self.create_publisher(String, 'serial_pta', 10)
        self.bridge = CvBridge()
        self.cap = None
        self.timer = None
        self.cv_image = None
        self.model = YOLO('./models/yolov8n-ep200-frz-d1.pt', task='segment')
        self.running_average = RunningAverage()
        self.create_blank_image()
        self.record = False
        self.writer_orig = None
        self.writer_inferenced = None

    def start_camera(self, device_index):
        if self.cap is not None:
            self.cap.release()
        self.cap = cv2.VideoCapture(device_index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        if self.timer is not None:
            self.timer.cancel()
        self.timer = self.create_timer(1.0 / 20.0, self.publish_camera_frame)
        if self.record:
            local_time = time.localtime()
            formatted_time = time.strftime('%Y-%m-%d-%H%M%S', local_time)
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self.writer_orig = cv2.VideoWriter(f'./videos/orig/{formatted_time}.mp4', fourcc, 20.0, (1920, 1080))
            self.writer_inferenced = cv2.VideoWriter(f'./videos/inferenced/{formatted_time}.mp4', fourcc, 20.0, (1920, 1080))

    def create_blank_image(self):
        self.cv_image = np.zeros((720, 1280, 3), dtype=np.uint8)
    
    def update_image_cam(self, img, pos):
        # Resize the incoming image to fit into the top-left quadrant (640x360)
        resized_img = cv2.resize(img, (640, 360))
        if pos == 0:
            self.cv_image[0:360, 0:640] = resized_img
        elif pos == 1:
            self.cv_image[0:360, 640:] = resized_img
        elif pos == 2:
            self.cv_image[360:, :640] = resized_img
        elif pos == 3:
            self.cv_image[360:, 640:] = resized_img

    def publish_camera_frame(self):
        if self.cap is None:
            return
        ret, frame = self.cap.read()
        if ret:
            self.update_image_cam(frame, 0)
            try:
                #img_inferenced, _, line1_ang, line2_ang = inf_angle(self.model, frame, self.running_average)
                #steering_value = -9999.0
                #print(frame.shape, img_inferenced.shape)
                #if line1_ang != None:
                #    steering_value = map_to_n_levels(line1_ang)
                #    self.publish_angle(steering_value)
                #elif line2_ang != None:
                #    steering_value = map_to_n_levels(line2_ang)
                #    self.publish_angle(steering_value)
                #img_inferenced = put_message(img_inferenced, 3, [f'rl_ang : {line1_ang}',f'll_ang : {line2_ang}', f'steer : {steering_value}'])
                #self.update_image_cam(img_inferenced, 1)
                #if self.record:
                #    self.writer_orig.write(frame)
                #    self.writer_inferenced.write(img_inferenced)
                img_inferenced, line1_ang, line2_ang = inf_angle_mainline(self.model, frame)
                steering_value = -9999.0
                if line1_ang != None:
                    steering_value = map_to_n_levels(line1_ang)
                    self.publish_angle(steering_value)
                elif line2_ang != None:
                    steering_value = map_to_n_levels(line2_ang)
                    self.publish_angle(steering_value)
                img_inferenced = put_message(img_inferenced, 3, [f'rl_ang : {line1_ang}',f'll_ang : {line2_ang}', f'steer : {steering_value}'])
                
                if self.record:
                    self.writer_orig.write(frame)
                    self.writer_inferenced.write(img_inferenced)
                self.update_image_cam(img_inferenced, 1)
            except CvBridgeError as e:
                self.get_logger().error(f"Error converting OpenCV image to ROS Image message: {e}")
    def publish_angle(self, value):
        data = 'angle%02d;' % value
        msg = String()
        msg.data = data
        self.pta_publisher.publish(msg)

    def start_video(self, video_path):
        if self.cap is not None:
            self.cap.release()
        self.cap = cv2.VideoCapture(video_path)
        if self.timer is not None:
            self.timer.cancel()
        self.timer = self.create_timer(1.0 / 20.0, self.publish_camera_frame)
        if self.record:
            local_time = time.localtime()
            formatted_time = time.strftime('%Y-%m-%d-%H%M%S', local_time)
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self.writer_orig = cv2.VideoWriter(f'./videos/orig/{formatted_time}.mp4', fourcc, 20.0, (1280, 720))
            self.writer_inferenced = cv2.VideoWriter(f'./videos/inferenced/{formatted_time}.mp4', fourcc, 20.0, (1280, 720))

    def destroy_node(self):
        super().destroy_node()
        if self.cap is not None:
            self.cap.release()
            
    def stop_cam(self):
        if self.cap is not None:
            self.cap.release()
        if self.timer is not None:
            self.timer.cancel()
        if self.record:
            self.writer_orig.release()
            self.writer_inferenced.release()

class LidarNode(Node):
    def __init__(self):
        super().__init__('lidar_node')
        self.publisher = self.create_publisher(String, 'lidar_data', 10)
        self.timer = None

    def start_lidar(self, port):
        self.port = port
        if self.timer is not None:
            self.timer.cancel()
        self.timer = self.create_timer(1.0 / 1.0, self.publish_lidar_data)

    def publish_lidar_data(self):
        # Dummy data for demonstration purposes
        data = "Lidar data from port " + self.port
        msg = String()
        msg.data = data
        self.publisher.publish(msg)

class ArdNode(Node):
    def __init__(self):
        super().__init__('ard_node')
        self.subscription_cam = self.create_subscription(
            String,
            'serial_pta',
            self.listener_pta,
            10)
        self.publisher = self.create_publisher(String, 'serial_atp', 10)
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
        # Dummy data for demonstration purposes
        data = "Arduino data from port " + self.port
        msg = String()
        msg.data = data
        self.publisher.publish(msg)
    
    def listener_pta(self, data):
        if self.port != '':
            self.serial_session.write(data.data.encode())

class MainNode(Node):
    def __init__(self):
        super().__init__('main_node')
        self.subscription_lidar = self.create_subscription(
            String,
            'lidar_data',
            self.listener_callback_lidar,
            10)
        self.subscription_serial = self.create_subscription(
            String,
            'serial_atp',
            self.listener_callback_serial,
            10)
        self.pta_publisher = self.create_publisher(String, 'serial_pta', 10)
        self.bridge = CvBridge()
        self.cv_image = None
        self.create_blank_image()
        self.model = YOLO('./last.engine', task='segment')

    def create_blank_image(self):
        self.cv_image = np.zeros((720, 1280, 3), dtype=np.uint8)

    def listener_callback_lidar(self, data):
        # TODO: Add code to update lower-left quadrant of self.cv_image with lidar data
        pass

    def listener_callback_serial(self, data):
        # TODO: Add code to handle serial data if needed
        pass

    def update_image_cam(self, img, pos):
        # Resize the incoming image to fit into the top-left quadrant (640x360)
        resized_img = cv2.resize(img, (640, 360))
        if pos == 0:
            self.cv_image[0:360, 0:640] = resized_img
        elif pos == 1:
            self.cv_image[0:360, 640:] = resized_img
        elif pos == 2:
            self.cv_image[360:, :640] = resized_img
        elif pos == 3:
            self.cv_image[360:, 640:] = resized_img

    def publish_setting(self):
        data = 'setting;'
        msg = String()
        msg.data = data
        self.pta_publisher.publish(msg)
        
    def publish_speed(self, value):
        data = 'speed'
        if value >= 0:
            data += '+'
        else:
            data += '-'
        data += '%03d;' % value
        msg = String()
        msg.data = data
        self.pta_publisher.publish(msg)
        
    def publish_angle(self, value):
        data = 'angle%02d;' % value
        msg = String()
        msg.data = data
        self.pta_publisher.publish(msg)
        
class MyApp(QWidget):
    def __init__(self, ros_nodes):
        super().__init__()
        self.main_node, self.cam_node, self.lidar_node, self.ard_node = ros_nodes
        self.cam_conn, self.lidar_conn, self.ard_conn = False, False, False
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
        self.settingAngleButton = QPushButton('Car Angle Setting')
        self.TestStartButton.clicked.connect(self.TestStartButtonClicked)
        self.speed255Button.clicked.connect(self.speed255ButtonClicked)
        self.speed150Button.clicked.connect(self.speed150ButtonClicked)
        self.speed080Button.clicked.connect(self.speed080ButtonClicked)
        self.speed000Button.clicked.connect(self.speed000ButtonClicked)
        self.settingAngleButton.clicked.connect(self.settingAngleButtonClicked)
        rightBottomLayout = QVBoxLayout()
        rightBottomLayout.addWidget(self.TestStartButton)
        rightBottomLayout.addWidget(self.speed255Button)
        rightBottomLayout.addWidget(self.speed150Button)
        rightBottomLayout.addWidget(self.speed080Button)
        rightBottomLayout.addWidget(self.speed000Button)
        rightBottomLayout.addWidget(self.settingAngleButton)
        
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
        while True:
            cap = cv2.VideoCapture(index)
            if not cap.read()[0]:
                break
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
        self.cam_node.record = self.checkboxRecord.isChecked()

    def selectCamDevice(self, comboBox, selectButton):
        if not self.cam_conn:
            index = int(comboBox.currentText())
            comboBox.setEnabled(False)
            self.cam_node.start_camera(index)
            selectButton.setText("Disconnect")
            self.cam_conn = True
        else:
            comboBox.setEnabled(True)
            self.cam_node.stop_cam()
            selectButton.setText("Connect")
            self.cam_conn = False

    def selectLidarDevice(self, comboBox, selectButton):
        port = comboBox.currentText()
        selectButton.setEnabled(False)
        comboBox.setEnabled(False)
        self.lidar_node.start_lidar(port)

    def selectArdDevice(self, comboBox, selectButton):
        port = comboBox.currentText()
        selectButton.setEnabled(False)
        comboBox.setEnabled(False)
        self.ard_node.start_arduino(port)

    def TestStartButtonClicked(self):
        if self.checkboxTest.isChecked():
            if self.cam_node.cap is not None:
                self.cam_node.stop_cam()
            video_path = './videos/orig/2024-07-03-060729.mp4'
            self.cam_node.start_video(video_path)
            

    def updateImage(self):
        if self.cam_node.cv_image is not None:
            self.showImage(self.cam_node.cv_image)

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
        
        

def main(args=None):
    rclpy.init(args=args)
    main_node = MainNode()
    cam_node = CamNode()
    lidar_node = LidarNode()
    ard_node = ArdNode()

    nodes = [main_node, cam_node, lidar_node, ard_node]
    executor = MultiThreadedExecutor()
    for node in nodes:
        executor.add_node(node)

    app = QApplication(sys.argv)
    ex = MyApp(nodes)
    
    # Spin ROS node in a separate thread to avoid blocking the main thread
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
