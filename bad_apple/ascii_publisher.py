#usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os
import glob

class AsciiPublisher(Node):
    def __init__(self):
        super().__init__('ascii_publisher')
        self.publisher_ = self.create_publisher(String, 'frames_ascii', 10)
        timer_period = 0.04  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Define the path to the frames-ascii folder
        package_share_directory = self.get_package_share_directory('bad_apple')
        self.frames_ascii_path = os.path.join(package_share_directory, 'frames-ascii')
        
        # Get list of all txt files in the frames-ascii folder
        self.file_list = sorted(glob.glob(os.path.join(self.frames_ascii_path, '*.txt')))
        self.current_file_index = 0

        if not self.file_list:
            self.get_logger().error(f'No txt files found in {self.frames_ascii_path}')
        else:
            self.get_logger().info(f'Found {len(self.file_list)} txt files in {self.frames_ascii_path}')

    def timer_callback(self):
        if not self.file_list:
            return

        if self.current_file_index >= len(self.file_list):
            self.current_file_index = 0  # Loop back to the first file

        current_file = self.file_list[self.current_file_index]
        try:
            with open(current_file, 'r') as file:
                content = file.read()
                msg = String()
                msg.data = content
                self.publisher_.publish(msg)
                self.get_logger().debug(f'Published content of {os.path.basename(current_file)}')
        except Exception as e:
            self.get_logger().error(f'Failed to read {current_file}: {e}')

        self.current_file_index += 1

    def get_package_share_directory(self, package_name):
        import ament_index_python.packages
        try:
            return ament_index_python.packages.get_package_share_directory(package_name)
        except Exception as e:
            self.get_logger().error(f'Could not find package {package_name}: {e}')
            return ''

def main(args=None):
    rclpy.init(args=args)
    ascii_publisher = AsciiPublisher()
    rclpy.spin(ascii_publisher)
    ascii_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
