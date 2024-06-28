import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from my_custom_msgs.msg import LidarData, LidarDatas
from cv_bridge import CvBridge
from lidar.rplidar import RPLidar, _process_scan
import serial
import serial.tools.list_ports
import cv2
import time
import math
import numpy as np

SERIAL_BAUDRATE = 9600
ARDUINO_PORT = ''
LIDAR_PORT = ''


class ArduinoCom(Node):
    def __init__(self):
        # node for arduino communication
        # 1. create serial communication sesison
        # 2. setup subscribe topic(pc to arduino message)
        #    when message received, process message and send via serial port
        # 3. setup publish topic(arduino to pc message)
        #    spin the checking process, when serial message received, send that via publishing topic
        super().__init__('arduino_com')
        self.cnt = 0
        # step 1.
        self.serial_session = None
        while self.serial_session == None:
            self.serial_session = self.create_session()
        
        # step 2.
        self.subscription = self.create_subscription(String, 'serial_pta', self.pta_callback, 10)
        
        # step 3.
        timer_period = .1
        self.buffer = []
        self.publisher_ = self.create_publisher(String, 'serial_atp', 10)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
    def create_session(self):
        global SERIAL_BAUDRATE, ARDUINO_PORT, LIDAR_PORT
        print('[arduino Node] arduino serial session finding...')
        ports = serial.tools.list_ports.comports()
        available_ports = [port.device for port in ports if port.device != LIDAR_PORT]
        serial_port = ''
        for port in reversed(available_ports):
            try:
                ser = serial.Serial(port, baudrate=SERIAL_BAUDRATE)
                print(f'Checking port {port}',end='')
                cnt = 0
                while ser.in_waiting == 0 and cnt < 5:
                    print('.',end='')
                    ser.write(b'arduino\n')
                    cnt += 1
                    time.sleep(1)
                response = ser.read(ser.in_waiting)
                print(response)
                if b'me' in response:
                    print(f'[arduino Node] serial port detected {port}')
                    ARDUINO_PORT = port
                    return ser
            except (serial.SerialException, OSError) as e:
                print(f'[arduino Node] Error with port {port}: {e}')
        print('[arduino Node] No arduino port is detected')
        return None
        #port = [port.device for port in serial.tools.list_ports.comports() if port.device != LIDAR_PORT][0]
        #self.serial_session = serial.Serial(port, baudrate=SERIAL_BAUDRATE, timeout=1)
        
    def pta_callback(self, msg):
        self.serial_session.write(msg.data.encode())
    
    def timer_callback(self):
        if self.serial_session is None:
            self.serial_session = self.create_session()
        else:
            while self.serial_session.in_waiting > 0:
                byte = self.serial.read(1)
                self.buffer.append(byte)
                print(byte)
                if byte == b'\n':
                    message = b''.join(self.buffer).decode('utf-8').strip()
                    print(message)
                    self.buffer = []
                    msg = String()
                    msg.data = message
                    self.publisher_.publish(msg)
    
    
class LidarCom(Node):
    def __init__(self):
        global LIDAR_PORT, ARDUINO_PORT
        super().__init__('lidar_com')
        self.publisher_ = self.create_publisher(LidarDatas, 'lidar_data', 10)
        timer_period = 1e-4
        ports = serial.tools.list_ports.comports()
        available_ports = [port.device for port in ports if port.device != ARDUINO_PORT]
        self.lidar = None
        for port in reversed(available_ports):
            try:
                ser = serial.Serial(port, 115200)
                ser.read(ser.in_waiting)
                ser.close()
                time.sleep(3)
                self.lidar = RPLidar(port)
                h = self.lidar.get_health()
                print(f'[lidar Node] found lidar sensor {port}')
                LIDAR_PORT = port
                break
            except:
            	self.lidar.disconnect()
            	print(f'[lidar Node] {port} is not lidar sensor')
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.lidar.start_motor()
        self.lidar.start('normal')
        self.frame_list = LidarDatas()
        
    def timer_callback(self):
        raw = self.lidar._read_response(self.lidar.scanning[1])
        new_scan, _, theta, r = _process_scan(raw)
        r = r / 10
        data = LidarData()
        data.angle = theta
        data.distance = r
        self.frame_list.data.append(data)
        if new_scan:
            self.publisher_.publish(self.frame_list)
            print(len(self.frame_list.data), time.time())
            self.frame_list = LidarDatas()

class LidarCom1(Node):
    def __init__(self):
        super().__init__('lidar_com')
        self.subscription = self.create_subscription(
            LidarData,
            'lidar_data',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        
    def listener_callback(self, msg):
        # receive data and visualize
        pass

class CameraCom(Node):
    def __init__(self, camera_num=0):
        # node for camera communication
        super().__init__('camera_com')
        self.publisher_ = self.create_publisher(Image, f'cam_data_{camera_num}', 10)
        timer_period = 0.05  # 10 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.cap = cv2.VideoCapture(camera_num)  # Change the index if needed
        self.bridge = CvBridge()

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher_.publish(msg)
            #print('[camera Node] data sended')
        else:
            print('Failed to read image from camera')


class CameraCom1(Node):
    def __init__(self, camera_num=0):
        # node for camera communication
        # 1. create serial communication sesison
        # 2. setup subscribe topic(pc to arduino message)
        #    when message received, process message and send via serial port
        # 3. setup publish topic(arduino to pc message)
        #    spin the checking process, when serial message received, send that via publishing topic
        super().__init__('camera_com')
        self.subscription = self.create_subscription(
            Image,
            'image',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        cv2.imshow('Received Image', frame)

