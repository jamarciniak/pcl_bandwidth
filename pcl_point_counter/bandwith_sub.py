import rclpy
from rclpy.node import Node
from point_cloud_interfaces.msg._compressed_point_cloud2 import CompressedPointCloud2
from sensor_msgs.msg import PointCloud2
import time
import pandas as pd
from rclpy.qos import QoSProfile, qos_profile_sensor_data
import sys
import os 

class BandwidthSubscriber(Node):
    def __init__(self):
        super().__init__('bandwidth_subscriber')
        self.get_logger().info("Bandwidth subscriber node started")
        self.declare_parameter('mode', rclpy.Parameter.Type.STRING) 
        self.declare_parameter('dataset_name', rclpy.Parameter.Type.STRING)
        # self.mode = sys.argv[1] if len(sys.argv) > 1 else ''
        self.mode = self.get_parameter('mode').value
        self.dataset_name = self.get_parameter('dataset_name').value
        self.get_logger().info(f"Mode: {self.mode}")
        self.get_logger().info(f"Dataset name: {self.dataset_name}")
        topic_suffix = f'/{self.mode}' if self.mode !='raw' else ''
        self.get_logger().info(f"Mode:{self.mode},topic: {f'/pct/point_cloud{topic_suffix} '}")
        self.subscription = self.create_subscription(
            CompressedPointCloud2 if self.mode != 'raw' else PointCloud2,
            f'/pct/point_cloud{topic_suffix}',
            self.listener_callback,
            qos_profile_sensor_data)
        
        
        self.data = {'timestamp': [], 'data': [], 'index': []}
        self.data_per_msg = {'timestamp': [], 'data': [], 'index': []}
        self.start_time = time.time()
        self.index = 0
        self.data_since_last_calculation = 0
        self.calculation_interval = 1  # seconds
        self.last_calculation_time = self.start_time
        
        self.name_suffix = f'_{self.mode}'
        self.filename = f'results/{self.dataset_name}/{self.dataset_name}{self.name_suffix}_bandwidth_data.csv'
        self.filename_per_msg = f'results/{self.dataset_name}/{self.dataset_name}{self.name_suffix}_bandwidth_per_msg.csv'

    def listener_callback(self, msg):
        current_time = time.time()
        elapsed_time = current_time - self.start_time
        data_path = msg.compressed_data if self.mode != 'raw' else msg.data
        bandwidth_per_msg = (len(data_path)*8) / elapsed_time  # Bytes per second
        self.update_data_per_msg(bandwidth_per_msg)
        

        self.data_since_last_calculation += len(data_path)
        # self.get_logger().info(f"Bandwidth: {bandwidth} Bytes per second")
        self.start_time = current_time

        # Check if it's time to perform calculation
        if current_time - self.last_calculation_time >= self.calculation_interval:
            self.calculate_bandwidth(current_time)
            self.last_calculation_time = current_time
            self.data_since_last_calculation = 0
            
        if current_time - self.last_calculation_time >= 1:
            self.save_data_to_dataframe('bandwidth_data.csv')

    def calculate_bandwidth(self,current_time):
        self.index += 1
        
        bandwidth = (self.data_since_last_calculation * 8) / self.calculation_interval  # Bytes per second
        self.data['timestamp'].append(current_time)
        self.data['data'].append(bandwidth)
        self.data['index'].append(self.index)
        
    def update_data_per_msg(self, bandwidth_per_msg):
        self.data_per_msg['timestamp'].append(time.time())
        self.data_per_msg['data'].append(bandwidth_per_msg)
        self.data_per_msg['index'].append(self.index)

    def save_data_to_dataframe(self):
        os.makedirs(os.path.dirname(self.filename), exist_ok=True)
        df = pd.DataFrame(self.data)
        df.to_csv(self.filename, index=False)
        self.get_logger().info(f"Data saved to {self.filename}")
        
    def save_data_per_msg_to_dataframe(self):
        os.makedirs(os.path.dirname(self.filename_per_msg), exist_ok=True)
        df = pd.DataFrame(self.data_per_msg)
        df.to_csv(self.filename_per_msg, index=False)
        self.get_logger().info(f"Data saved to {self.filename_per_msg}")

def main():
    rclpy.init()
    bandwidth_subscriber = BandwidthSubscriber()
    try:
        rclpy.spin(bandwidth_subscriber)
    except KeyboardInterrupt:
        print("KeyboardInterrupt detected. Saving data...")
        bandwidth_subscriber.save_data_to_dataframe()
        bandwidth_subscriber.save_data_per_msg_to_dataframe()
        print("Data saved. Shutting down...")
    finally:
        bandwidth_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
