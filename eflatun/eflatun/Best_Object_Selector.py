import rclpy
from rclpy.node import Node
from eflatun_msgs.msg import TrackedObjectArray, TrackedObject
from typing import Tuple
from eflatun import Object


class BestObjectSelector(Node):

    def __init__(self):
        super().__init__('best_object_selector')

        # Configuration values
        self.frame_center = (1920 / 2, 1080 / 2)
        self.x_range = (1920 / 4, 3 * 1920 / 4)
        self.y_range = (1080 / 10, 9 * 1080 / 10)
        self.min_width = 80
        self.min_height = 40

        self.subscription = self.create_subscription(
            TrackedObjectArray,
            '/tracker/tracked_objects',
            self.listener_callback,
            2)

        self.publisher = self.create_publisher(
            TrackedObject,
            '/tracker/best_object',
            2)

        self.subscription  # prevent unused variable warning

    def calculate_score(self, tracked_object: Object) -> float:
        dist_to_center = ((tracked_object.center_x - self.frame_center[0]) ** 2 +
                          (tracked_object.center_y - self.frame_center[1]) ** 2) ** 0.5

        score = (tracked_object.width * 0.5 + tracked_object.height * 0.5 +
                 tracked_object.age * 1.0 - dist_to_center * 0.2)
        return score

    def listener_callback(self, msg: TrackedObjectArray) -> None:
        best_object = None
        best_score = -1

        for tracked_object_msg in msg.detections:
            tracked_object = Object(tracked_object_msg)

            if not tracked_object.has_desired_size(self.min_width, self.min_height):
                continue

            if not tracked_object.is_within_range(self.x_range, self.y_range):
                continue

            score = self.calculate_score(tracked_object)

            if score > best_score:
                best_score = score
                best_object = tracked_object

            if tracked_object.age == 120:
                self.get_logger().info(f"Object {tracked_object.unique_id} has reached age 120")

        if best_object:
            self.get_logger().info(f"Best object: {best_object}")
            best_object_msg = best_object.to_ros_message()
            self.publisher.publish(best_object_msg)
        else:
            self.get_logger().info("No object meets the criteria")


def main(args=None) -> None:
    rclpy.init(args=args)

    best_object_selector = BestObjectSelector()

    rclpy.spin(best_object_selector)

    best_object_selector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
