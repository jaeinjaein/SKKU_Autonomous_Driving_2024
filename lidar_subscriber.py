import rclpy
from rclpy.node import Node
from my_custom_msgs.msg import LidarData, LidarDatas
import cv2
import math
import numpy as np

class LidarSubscriber(Node):

    def __init__(self):
        super().__init__('lidar_subscriber')
        self.subscription = self.create_subscription(
            LidarDatas,
            'scan',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info('Lidar subscriber node started.')
        cv2.namedWindow("Lidar View", cv2.WINDOW_NORMAL)

    def listener_callback(self, msg):
        # LiDAR 데이터 출력
        self.get_logger().info(f'Received LiDAR data: {len(msg.data)}')  # 예시로 처음 10개의 거리 데이터를 출력
        self.visualize_lidar_data(msg.data)
        
    def visualize_lidar_data(self, datas):
        # LiDAR 데이터를 시각화하는 로직
        img = np.zeros((500, 500, 3), dtype=np.uint8)
        center = (250, 250)
        
        for data in datas:
            angle, distance = data.angle, data.distance / 10
            if distance > 0:
                x = int(center[0] + (distance * 100.0) * math.cos(angle))
                y = int(center[1] + (distance * 100.0) * math.sin(angle))
                cv2.circle(img, (x, y), 2, (0, 255, 0), -1)
        
        cv2.imshow("Lidar View", img)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    lidar_subscriber = LidarSubscriber()
    
    try:
        rclpy.spin(lidar_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        # 노드를 종료합니다.
        lidar_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

