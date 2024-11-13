import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import os

class SignalSource(Node):
    def __init__(self):
        super().__init__('signal_source')
        self.publisher_ = self.create_publisher(Float32, '/signal_raw', 10)
        timer_period = 0.05 
        self.timer = self.create_timer(timer_period, self.publish_signal)
        self.index = 0

        # Had some trouble with the file path initially
        package_path = os.path.join(os.getenv('AMENT_PREFIX_PATH').split(':')[0], 'share', 'signal_processing_pkg')
        file_path = os.path.join(package_path, 'signal_raw.dat')

        try:
            with open(file_path, 'r') as file:
                self.data = [float(line.strip()) for line in file]
            self.get_logger().info(f"Loaded {len(self.data)} samples from {file_path}")
        except Exception as e:
            self.get_logger().error(f"File not found or unable to read file: {e}")
            self.data = []

    def publish_signal(self):
        if self.index < len(self.data):
            value = self.data[self.index]
            self.publisher_.publish(Float32(data=value))
            self.index += 1
        else:
            self.index = 0  # REseting index to loop

def main(args=None):
    rclpy.init(args=args)
    node = SignalSource()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
