import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

class TestYAMLParams(Node):

    def __init__(self):
        super().__init__('your_amazing_node')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('bool_value', Parameter.Type.BOOL),
            ])

def main(args=None):
    rclpy.init(args=args)
    node = TestYAMLParams()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()