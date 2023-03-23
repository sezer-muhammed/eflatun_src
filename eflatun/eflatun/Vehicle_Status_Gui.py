import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, qos_profile_sensor_data

from sensor_msgs.msg import BatteryState, NavSatFix
from geometry_msgs.msg import TwistStamped

class MavrosSubscriber(Node):

    def __init__(self):
        super().__init__('mavros_subscriber')

        # Subscribe to battery topic
        self.create_subscription(
            BatteryState,
            '/mavros/battery',
            self.battery_callback,
            qos_profile_sensor_data
        )
        self.get_logger().info("Subscribed to /mavros/battery")

        # Subscribe to GPS topic
        self.create_subscription(
            NavSatFix,
            '/mavros/global_position/global',
            self.gps_callback,
            qos_profile_sensor_data
        )
        self.get_logger().info("Subscribed to /mavros/global_position/global")

        # Subscribe to local position velocity topic
        self.create_subscription(
            TwistStamped,
            '/mavros/local_position/velocity_local',
            self.velocity_callback,
            qos_profile_sensor_data
        )
        self.get_logger().info("Subscribed to /mavros/local_position/velocity_local")

    def battery_callback(self, msg: BatteryState):
        self.get_logger().info(f"Battery: {msg.voltage} V, {msg.percentage * 100}%")

    def gps_callback(self, msg: NavSatFix):
        self.get_logger().info(f"GPS: Latitude={msg.latitude}, Longitude={msg.longitude}, Altitude={msg.altitude}")

    def velocity_callback(self, msg: TwistStamped):
        self.get_logger().info(f"Velocity: x={msg.twist.linear.x}, y={msg.twist.linear.y}, z={msg.twist.linear.z}")

def main(args=None):
    rclpy.init(args=args)
    mavros_subscriber = MavrosSubscriber()
    rclpy.spin(mavros_subscriber)
    mavros_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
