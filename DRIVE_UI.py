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
from inference import inference_image, RunningAverage, inf_angle
from ultralytics import YOLO
import matplotlib.pyplot as plt
import time
import serial

test_list = []
ard_list = []

def map_to_n_levels(value):
    if value < -60:
        value = -60
    elif value > 60:
        value = 60

    # Map the value to the range 0 to 14
    mapped_value = int((value + 60) / 120 * 14)

    return mapped_value + 1

class CamNode(Node):
    def __init__(self):
        super().__init__('cam_node')
     
        self.publisher = self.create_publisher(Image, 'cam_img', 10)
        self.pta_publisher = self.create_publisher(String, 'serial_pta', 10)
        self.bridge = CvBridge()
        self.cap = None
        self.timer = None
        
        self.model = YOLO('./last.engine', task='segment')
        self.running_average = RunningAverage()

    def start_camera(self, device_index):
        if self.cap is not None:
            self.cap.release()
        self.cap = cv2.VideoCapture(device_index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        if self.timer is not None:
            self.timer.cancel()
        self.timer = self.create_timer(1.0 / 10.0, self.publish_camera_frame)

    def publish_camera_frame(self):
        if self.cap is None:
            return
        ret, frame = self.cap.read()
        if ret:
            try:
                #msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                img_inferenced, _, line1_ang, line2_ang = inf_angle(self.model, frame, self.running_average)
                steering_value = -9999.0
                if line1_ang != None:
                    steering_value = map_to_n_levels(line1_ang)
                    self.publish_angle(steering_value)
                elif line2_ang != None:
                    steering_value = map_to_n_levels(line2_ang)
                    self.publish_angle(steering_value)
                
                #self.publisher.publish(msg)
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
        self.timer = self.create_timer(1.0 / 30.0, self.publish_camera_frame)

    def destroy_node(self):
        super().destroy_node()
        if self.cap is not None:
            self.cap.release()

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
        self.subscription_cam = self.create_subscription(
            Image,
            'cam_img',
            self.listener_callback_cam,
            10)
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
        self.running_average = RunningAverage()

    def create_blank_image(self):
        self.cv_image = np.zeros((720, 1280, 3), dtype=np.uint8)

    def listener_callback_cam(self, data):
        global test_list
        try:
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.update_image_cam(img, 0)
            img_inferenced, _, line1_ang, line2_ang = inf_angle(self.model, img, self.running_average)
            steering_value = -9999.0
            if line1_ang != None:
                steering_value = map_to_n_levels(line1_ang)
                self.publish_angle(steering_value)
            elif line2_ang != None:
                steering_value = map_to_n_levels(line2_ang)
                self.publish_angle(steering_value)
            #img_inferenced, result_degree = inference_image(self.model, img)
            self.update_image_cam(img_inferenced, 1)
            #test_list.append(result_degree)
            #before_value = test_list[len(test_list)-2]
            #if before_value != -1000.0:  # trash value
            #    diff = abs(result_degree - before_value)
            #    if diff > 30.0:
            #        result_degree = -1000.0
            #test_list[len(test_list)-1] = result_degree
            #if result_degree != -1000.0:
            #    steering_value = map_to_n_levels(result_degree)
            #    self.publish_angle(steering_value)
            #cv2.imwrite(f'./dataset_img/{time.time_ns()}.jpg', img)
            
        except CvBridgeError as e:
            self.get_logger().error(f"Error converting ROS Image message to OpenCV: {e}")

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
        self.initUI()

    def initUI(self):
        layout = QGridLayout()

        # Left top checkbox group
        self.leftTopGroup = QGroupBox('Modes')
        self.checkbox1 = QCheckBox('DRIVE')
        self.checkbox2 = QCheckBox('AV/TR')
        self.checkbox3 = QCheckBox('PARK')
        self.checkboxTest = QCheckBox('TEST')
        self.checkbox1.stateChanged.connect(lambda: self.modeChanged(self.checkbox1))
        self.checkbox2.stateChanged.connect(lambda: self.modeChanged(self.checkbox2))
        self.checkbox3.stateChanged.connect(lambda: self.modeChanged(self.checkbox3))
        self.checkboxTest.stateChanged.connect(lambda: self.modeChanged(self.checkboxTest))
        leftTopLayout = QVBoxLayout()
        leftTopLayout.addWidget(self.checkbox1)
        leftTopLayout.addWidget(self.checkbox2)
        leftTopLayout.addWidget(self.checkbox3)
        leftTopLayout.addWidget(self.checkboxTest)
        self.leftTopGroup.setLayout(leftTopLayout)
        
        # Right top driver select group
        self.rightTopGroup = QGroupBox('Driver Select')
        rightTopLayout = QVBoxLayout()

        self.camComboBox = QComboBox()
        self.camSelectButton = QPushButton('Select')
        self.createDeviceSelectionLayout(rightTopLayout, 'CAM Device', self.populateCamComboBox, self.camComboBox, self.camSelectButton, self.selectCamDevice)

        self.lidarComboBox = QComboBox()
        self.lidarSelectButton = QPushButton('Select')
        self.createDeviceSelectionLayout(rightTopLayout, 'Lidar Device', self.populateSerialComboBox, self.lidarComboBox, self.lidarSelectButton, self.selectLidarDevice)

        self.ardComboBox = QComboBox()
        self.ardSelectButton = QPushButton('Select')
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
        self.startButton = QPushButton('Start')
        self.startButton.clicked.connect(self.startButtonClicked)
        self.rearstartButton = QPushButton('RearStart')
        self.rearstartButton.clicked.connect(self.rearstartButtonClicked)
        rightBottomLayout = QVBoxLayout()
        rightBottomLayout.addWidget(self.startButton)
        rightBottomLayout.addWidget(self.rearstartButton)
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

    def selectCamDevice(self, comboBox, selectButton):
        index = int(comboBox.currentText())
        selectButton.setEnabled(False)
        comboBox.setEnabled(False)
        self.cam_node.start_camera(index)

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

    def startButtonClicked(self):
        global test_list
        if len(test_list) > 1:
            res = np.array(test_list)
            np.save(f'./{time.time_ns()}_drive.npy', res)
            #differences = [j-i for i, j in zip(test_list[:-1], test_list[1:])]
            #plt.plot(test_list, label='Data')
            #plt.plot(differences, label='Differences', linestyle='--', marker='o')
            #plt.grid(True)
            #plt.show()
            test_list = [-1000.0]
        if self.checkboxTest.isChecked():
            video_path = './yolov8-seg/1_1415.mp4'
            self.cam_node.start_video(video_path)

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
        
    def rearstartButtonClicked(self):
        self.main_node.publish_speed(150)
        

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
