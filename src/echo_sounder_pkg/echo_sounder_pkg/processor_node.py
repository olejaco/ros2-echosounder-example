import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class DepthProcessor(Node):
    def __init__(self):
        super().__init__('depth_processor')
        self.subscription = self.create_subscription(
            Float32,
            'raw_depth',
            self.depth_callback,
            10)
        self.publisher_ = self.create_publisher(Float32, 'processed_depth', 10)
        self.get_logger().info('Depth Processor Node has been started')

    def depth_callback(self, msg):
        processed_depth = msg.data * 0.9  # Simple processing: 10% error correction
        processed_msg = Float32()
        processed_msg.data = processed_depth
        self.publisher_.publish(processed_msg)
        self.get_logger().info(f'Raw depth: {msg.data}, Processed depth: {processed_depth}')

def main(args=None):
    rclpy.init(args=args)
    depth_processor = DepthProcessor()
    rclpy.spin(depth_processor)
    depth_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()