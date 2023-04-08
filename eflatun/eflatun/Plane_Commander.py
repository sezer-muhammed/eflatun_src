import rclpy
from rclpy.node import Node
from eflatun_msgs.msg import TrackedObject
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandLong

from rclpy.qos import qos_profile_sensor_data


class ObjectFollower(Node):

    def __init__(self):
        super().__init__('object_follower')
        self.subscription = self.create_subscription(TrackedObject, '/tracker/best_detection', self.listener_callback,
                                                     qos_profile_sensor_data)
        self.pose_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 3)
        self.command_srv = self.create_client(CommandLong, '/mavros/cmd/command')

    def listener_callback(self, msg: TrackedObject):
        # Proportional control gains
        k_roll = 0.1
        k_pitch = 0.1

        # Normalize the coordinates
        normalized_x = msg.center_x / 1920
        normalized_y = msg.center_y / 1080

        # Calculate roll and pitch commands based on object's position
        roll_command = k_roll * (normalized_x - 0.5)
        pitch_command = k_pitch * (normalized_y - 0.5)

        # Construct and send the PoseStamped message
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.orientation.x = roll_command
        pose_msg.pose.orientation.y = pitch_command
        self.pose_pub.publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)
    object_follower = ObjectFollower()
    rclpy.spin(object_follower)
    object_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
