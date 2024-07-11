from cv_bridge import CvBridge
import cv2

# this code is for providing camera image to main node (UI)
# (from GPT)

class LidarPublisher(Node):
    def __init__(self, serial_device):
        super().__init__('lidar_publisher')
        self.publisher_ = self.create_publisher(LidarData, 'lidar_data', 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
    
    def timer_callback(self):
        msg = LidarData()
        msg.angle = 45.0
        msg.distance = 10.0
        self.publisher_.publish(msg)
        print(f'Publishing: angle={msg.angle}, distance={msg.distance}')

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(Image, 'image', 10)
        timer_period = 0.1  # 10 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.cap = cv2.VideoCapture(0)  # Change the index if needed
        self.bridge = CvBridge()

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher_.publish(msg)
        else:
            self.get_logger().error('Failed to read image from camera')

#def main(args=None):
#    rclpy.init(args=args)
#    image_publisher = ImagePublisher()
#    rclpy.spin(image_publisher)
#    image_publisher.destroy_node()
#    rclpy.shutdown()

#if __name__ == '__main__':
#    main()

