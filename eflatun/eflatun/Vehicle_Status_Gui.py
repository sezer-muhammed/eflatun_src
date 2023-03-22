import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

import json

class MyNode(Node):
    def __init__(self):
        super().__init__('vehicle_status_gui')

        self.declare_parameters(namespace='',
                                parameters=[
                                    ('log_level', Parameter.Type.STRING),
                                    ('topics', Parameter.Type.STRING_ARRAY),
                                    ('frame_rate', Parameter.Type.INTEGER)
                                ])

        self.params = {
            'log_level': self.get_parameter('log_level').value,
            'topics': self.get_parameter('topics').value,
            'frame_rate': self.get_parameter('frame_rate').value
        }

        self.get_logger().info(json.dumps(self.params, sort_keys=True, indent=4))


def main(args=None):

    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
