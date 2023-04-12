import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandBool

class ArmingNode(Node):
    def __init__(self):
        super().__init__('arming_node')
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')

        self.arm_uav()

    def arm_uav(self):
        self.get_logger().info('Arming the UAV...')
        req = CommandBool.Request()
        req.value = True

        while not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for the arming service to be available...')

        future = self.arming_client.call_async(req)

        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info('UAV armed successfully.')
            else:
                self.get_logger().error('Failed to arm the UAV.')
        else:
            self.get_logger().error('Arming service call failed.')

def main(args=None):
    rclpy.init(args=args)

    arming_node = ArmingNode()

    arming_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
