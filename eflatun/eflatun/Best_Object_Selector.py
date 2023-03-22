import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

import json

from eflatun_msgs.msg import TrackedObjectArray, TrackedObject
from typing import Tuple
from eflatun import Object


class BestObjectSelector(Node):

    def __init__(self):
        super().__init__('best_object_selector')

        self.declare_parameters(namespace='',
                                parameters=[
                                    ('frame_center.x', Parameter.Type.INTEGER),
                                    ('frame_center.y', Parameter.Type.INTEGER),
                                    ('x_range.min', Parameter.Type.INTEGER),
                                    ('x_range.max', Parameter.Type.INTEGER),
                                    ('y_range.min', Parameter.Type.INTEGER),
                                    ('y_range.max', Parameter.Type.INTEGER),
                                    ('min_width', Parameter.Type.INTEGER),
                                    ('min_height', Parameter.Type.INTEGER),
                                    ('score_variables.width_weight', Parameter.Type.DOUBLE),
                                    ('score_variables.height_weight', Parameter.Type.DOUBLE),
                                    ('score_variables.age_weight', Parameter.Type.DOUBLE),
                                    ('score_variables.dist_to_center_weight', Parameter.Type.DOUBLE)
                                ])

        self.params = {
            'frame_center': {
                'x': self.get_parameter('frame_center.x').value,
                'y': self.get_parameter('frame_center.y').value
            },
            'x_range': {
                'min': self.get_parameter('x_range.min').value,
                'max': self.get_parameter('x_range.max').value
            },
            'y_range': {
                'min': self.get_parameter('y_range.min').value,
                'max': self.get_parameter('y_range.max').value
            },
            'min_width': self.get_parameter('min_width').value,
            'min_height': self.get_parameter('min_height').value,
            'score_variables': {
                'width_weight': self.get_parameter('score_variables.width_weight').value,
                'height_weight': self.get_parameter('score_variables.height_weight').value,
                'age_weight': self.get_parameter('score_variables.age_weight').value,
                'dist_to_center_weight': self.get_parameter('score_variables.dist_to_center_weight').value
            }
        }
        self.get_logger().info(json.dumps(self.params, sort_keys=True, indent=4))

        self.subscription = self.create_subscription(
            TrackedObjectArray,
            '/tracker/tracked_objects',
            self.listener_callback,
            2)

        self.publisher = self.create_publisher(
            TrackedObject,
            '/tracker/best_object',
            2)


    def calculate_score(self, tracked_object: Object) -> float:
        dist_to_center = ((tracked_object.center_x - self.params["frame_center"]["x"]) ** 2 +
                          (tracked_object.center_y - self.params["frame_center"]["y"]) ** 2) ** 0.5

        score = (tracked_object.width * 0.5 + tracked_object.height * 0.5 +
                 tracked_object.age * 1.0 - dist_to_center * 0.2)
        return score

    def listener_callback(self, msg: TrackedObjectArray) -> None:
        best_object = None
        best_score = -1

        for tracked_object_msg in msg.detections:
            tracked_object = Object(tracked_object_msg)

            if not tracked_object.has_desired_size(self.params["min_width"], self.params["min_height"]):
                continue

            if not tracked_object.is_within_range((self.params["x_range"]["min"], self.params["x_range"]["max"]), (self.params["y_range"]["min"], self.params["y_range"]["max"])):
                continue

            score = self.calculate_score(tracked_object)

            if score > best_score:
                best_score = score
                best_object = tracked_object

            if tracked_object.age == 120:
                self.get_logger().info(f"Object {tracked_object.unique_id} has reached age 120")

        if best_object:
            best_object_msg = best_object.to_ros_message()
            self.publisher.publish(best_object_msg)
        else:
            pass


def main(args=None) -> None:
    rclpy.init(args=args)

    best_object_selector = BestObjectSelector()

    rclpy.spin(best_object_selector)

    best_object_selector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
