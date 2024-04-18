import rclpy
from rclpy.node import Node
from point_cloud_interfaces.msg._compressed_point_cloud2 import CompressedPointCloud2
from sensor_msgs.msg import PointCloud2
import time
import pandas as pd
from rclpy.qos import QoSProfile, qos_profile_sensor_data
import sys

class BandwidthSubscriber(Node):
    def __init__(self):
        super().__init__('bandwidth_subscriber')
        self.get_logger().info("Bandwidth subscriber node started")
        self.mode = sys.argv[1] if len(sys.argv) > 1 else ''
        topic_suffix = f'/{self.mode}' if self.mode !='' else ''
        self.get_logger().info(f"Mode:{self.mode},topic: {f'/pct/point_cloud{topic_suffix},,,'}")
        self.subscription = self.create_subscription(
            CompressedPointCloud2 if bool(self.mode) else PointCloud2,
            f'/pct/point_cloud{topic_suffix}',
            self.listener_callback,
            qos_profile_sensor_data)
        self.data = {'timestamp': [], 'data': [], 'index': []}
        self.start_time = time.time()
        self.index = 0

    def listener_callback(self, msg):
        self.get_logger().info("Received message")
        current_time = time.time()
        elapsed_time = current_time - self.start_time
        data_path = msg.compressed_data if bool(self.mode) else msg.data
        bandwidth = (len(data_path)*8) / elapsed_time  # Bytes per second
        self.data['timestamp'].append(current_time)
        self.data['data'].append(bandwidth)
        self.data['index'].append(self.index)
        self.get_logger().info(f"Bandwidth: {bandwidth} Bytes per second")
        self.start_time = current_time
        self.index += 1


    def save_data_to_dataframe(self, filename):
        df = pd.DataFrame(self.data)
        df.to_csv(filename, index=False)
        self.get_logger().info(f"Data saved to {filename}")

def main():
    rclpy.init()
    bandwidth_subscriber = BandwidthSubscriber()
    try:
        rclpy.spin(bandwidth_subscriber)
    except KeyboardInterrupt:
        print("KeyboardInterrupt detected. Saving data...")
        name_suffix = f'_{bandwidth_subscriber.mode}' if bandwidth_subscriber.mode != '' else ''
        filename = f'bandwidth_data{name_suffix}.csv'
        bandwidth_subscriber.save_data_to_dataframe(filename)
        print("Data saved. Shutting down...")
    finally:
        bandwidth_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
