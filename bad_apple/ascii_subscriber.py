#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class AsciiSubscriber(Node):
    def __init__(self):
        super().__init__('ascii_subscriber')
        self.subscription = self.create_subscription(
            String,
            'frames_ascii',
            self.listener_callback,
            10
        )
        self.subscription  # Prevent unused variable warning
        self.get_logger().info('ASCII Subscriber Node has been started.')

    def listener_callback(self, msg):
        self.get_logger().info(f'Received ASCII Frame:\n{msg.data}')

def main(args=None):
    rclpy.init(args=args)
    ascii_subscriber = AsciiSubscriber()
    rclpy.spin(ascii_subscriber)

    # Destroy the node explicitly
    ascii_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 