#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

# Filter coefficients from MATLAB
a1 = -1.8229
a2 = 0.83738
b1 = 0.0072251
b2 = 0.0036126

class LowPassFilter:
    def __init__(self):
        self.y = [0.0, 0.0]  # Previous outputs y[k-1], y[k-2]
        self.u = [0.0, 0.0]  # Previous inputs u[k-1], u[k-2]

    def do_filter_calc(self, u_current):
        # Calculate the current output using the difference equation
        y_current = -a1 * self.y[0] - a2 * self.y[1] + b1 * u_current + b2 * self.u[0]

        # Update the history of inputs and outputs
        self.y[1] = self.y[0]
        self.y[0] = y_current
        self.u[1] = self.u[0]
        self.u[0] = u_current

        return y_current

class LPFilterNode(Node):
    def __init__(self):
        super().__init__('lp_filter_node')
        self.filter = LowPassFilter()
        self.publisher = self.create_publisher(Float32, '/signal_filtered', 10)
        self.subscription = self.create_subscription(Float32, '/signal_raw', self.callback, 10)
        self.subscription  # Prevent unused variable warning

    def callback(self, msg):
        raw_value = msg.data
        filtered_value = self.filter.do_filter_calc(raw_value)
        self.get_logger().info(f"Raw: {raw_value}, Filtered: {filtered_value}")
        self.publisher.publish(Float32(data=filtered_value))

def main(args=None):
    rclpy.init(args=args)
    node = LPFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

