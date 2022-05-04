import rclpy
from rclpy.node import Node

class GateNode(Node):
    def __init__(self):
        super().__init__("rembrain_gate_ros2")
        self.get_logger().info("This node just says 'Hello'")

def main_func(args=None):
    rclpy.init(args=args)
    node = GateNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
