from rplidar import RPLidar
import math
from lidar.rplidar import RPLidar, _process_scan

class RPLidarNode(Node):

    def __init__(self):
        super().__init__('rplidar_node')
        self.publisher_ = self.create_publisher(LidarDatas, 'scan', 10)
        self.lidar = RPLidar('/dev/ttyUSB0')
        # 타이머 설정 (1초 간격)
        self.timer = self.create_timer(1e-4, self.timer_callback)
        
        self.get_logger().info('RPLidar node started.')

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

    def publish_scan(self, scan):
        datas = LidarDatas()
        for (_, angle, distance) in scan:
            data = LidarData()
            data.angle = angle
            data.distance = distance / 10
            datas.data.append(data)

        self.publisher_.publish(datas)
        self.get_logger().info('Published scan data')

def main(args=None):
    rclpy.init(args=args)
    rplidar_node = RPLidarNode()
    
    try:
        rclpy.spin(rplidar_node)
    except KeyboardInterrupt:
        pass
    finally:
        # 노드를 종료합니다.
        rplidar_node.lidar.stop()
        rplidar_node.lidar.disconnect()
        rplidar_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
