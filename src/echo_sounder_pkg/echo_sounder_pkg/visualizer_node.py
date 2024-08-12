import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class DepthVisualizer(Node):
    def __init__(self):
        super().__init__('depth_visualizer')
        self.subscription = self.create_subscription(
            Float32,
            'processed_depth',
            self.depth_callback,
            10)
        self.get_logger().info('Depth Visualizer Node has been started')
        self.max_depth = 100.0  # Maximum depth for visualization
        self.resolution = 20  # Number of levels in the ASCII art

    def depth_callback(self, msg):
        depth = msg.data
        normalized_depth = min(depth / self.max_depth, 1.0)
        level = int(normalized_depth * self.resolution)

        # Create ASCII art representation
        visualization = '|' + '=' * level + ' ' * (self.resolution - level) + '|'
        
        self.get_logger().info(f'Depth: {depth:.2f}m')
        self.get_logger().info(visualization)
        self.get_logger().info('-' * (self.resolution + 2))  # Separator line

def main(args=None):
    rclpy.init(args=args)
    depth_visualizer = DepthVisualizer()
    rclpy.spin(depth_visualizer)
    depth_visualizer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()