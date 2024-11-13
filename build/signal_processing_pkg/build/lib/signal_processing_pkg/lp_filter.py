#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

# Filter coefficients
a1 = -1.5610180758007182
a2 = 0.641351538057563
b1 = 0.02008336556421123
b2 = 0.04016673112842246

class LowPassFilter(Node):
    def __init__(self):
        super().__init__('lp_filter')
        self.y = [0, 0]
        self.u = [0, 0]
        self.publisher = self.create_publisher(Float32, 'signal_filtered', 10)
        self.subscription = self.create_subscription(Float32, 'signal_raw', self.callback, 10)

    def do_filter_calc(self, u_current):
        y_current = -a1 * self.y[0] - a2 * self.y[1] + b1 * u_current + b2 * self.u[0]
        self.y[1] = self.y[0]
        self.y[0] = y_current
        self.u[1] = self.u[0]
        self.u[0] = u_current
        return y_current

    def callback(self, msg):
        raw_value = msg.data
        filtered_value = self.do_filter_calc(raw_value)
        self.get_logger().info(f"Raw: {raw_value}, Filtered: {filtered_value}")
        self.publisher.publish(Float32(data=filtered_value))

def main(args=None):
    rclpy.init(args=args)
    node = LowPassFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
