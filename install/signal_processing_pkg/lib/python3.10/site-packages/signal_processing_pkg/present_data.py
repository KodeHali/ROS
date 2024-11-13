#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class PresentData(Node):
    def __init__(self):
        super().__init__('present_data')
        self.subscription = self.create_subscription(Float32, 'signal_filtered', self.callback, 10)

    def callback(self, msg):
        self.get_logger().info(f"Filtered: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = PresentData()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
