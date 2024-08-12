import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

import random

class EchoSounderSensor(Node):
    def __init__(self):
        super().__init__('echo_sounder_sensor')
        self.publisher_ = self.create_publisher(Float32, 'raw_depth', 10)
        self.timer = self.create_timer(1.0, self.publish_depth)
        self.get_logger().info('Echo-Sounder Sensor Node has been started')

    def publish_depth(self):
        msg = Float32()
        msg.data = random.uniform(0.0, 100.0)  # Simulated depth reading
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing depth: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    echo_sounder_sensor = EchoSounderSensor()
    rclpy.spin(echo_sounder_sensor)
    echo_sounder_sensor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()