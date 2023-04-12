import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.logging import LoggingSeverity
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import qos_profile_sensor_data

import json

from eflatun_msgs.msg import TrackedObjectArray, TrackedObject
from typing import Tuple
from eflatun.Object import Object


class BestObjectSelector(Node):

    def __init__(self):
        super().__init__('best_object_selector')

        self.declare_parameters(namespace='',
                                parameters=[
                                    ('log_level', Parameter.Type.STRING),
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
            'log_level': self.get_parameter('log_level').value,
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

        log_level_mapping = {
            'debug': LoggingSeverity.DEBUG,
            'info': LoggingSeverity.INFO,
            'warn': LoggingSeverity.WARN,
            'error': LoggingSeverity.ERROR,
            'fatal': LoggingSeverity.FATAL,
        }


        self.add_on_set_parameters_callback(self.on_parameter_change)

        log_level = log_level_mapping.get(self.params["log_level"], LoggingSeverity.INFO)
        self.get_logger().set_level(log_level)

        self.get_logger().info(json.dumps(self.params, sort_keys=True, indent=4))

        self.subscription = self.create_subscription(
            TrackedObjectArray,
            '/tracker/tracked_objects',
            self.listener_callback,
            qos_profile_sensor_data)

        self.publisher = self.create_publisher(
            TrackedObject,
            '/tracker/best_object',
            qos_profile_sensor_data)


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
                self.get_logger().debug(f"Object {tracked_object.unique_id} skipped due to size")
                continue

            if not tracked_object.is_within_range((self.params["x_range"]["min"], self.params["x_range"]["max"]), (self.params["y_range"]["min"], self.params["y_range"]["max"])):
                self.get_logger().debug(f"Object {tracked_object.unique_id} skipped due to range")
                continue

            score = self.calculate_score(tracked_object)
            self.get_logger().debug(f"Object {tracked_object.unique_id} score: {score}")

            if score > best_score:
                best_score = score
                best_object = tracked_object

            if 120 < tracked_object.age:
                self.get_logger().info(f"Object {tracked_object.unique_id} has reached age 120 with {tracked_object.age}")

        if best_object:
            best_object_msg = best_object.to_ros_message()
            self.publisher.publish(best_object_msg)
        else:
            self.get_logger().debug("No best object found")
            pass

    def on_parameter_change(self, params):
        # Update the parameter dictionary with the new values
        for param in params:
            try:
                param_name = param.name
                param_value = param.value

                param_name = param_name.split(".")
                params_temp = self.params
                for key in param_name[:-1]:
                    params_temp = params_temp[key]
                params_temp[param_name[-1]] = param_value
                self.get_logger().info('Parameter {} updated to {}'.format(param_name, param_value))
            except:
                self.get_logger().warning('Parameter {} cannot updated to {}'.format(param_name, param_value))
                return SetParametersResult(successful=False)
            
        return SetParametersResult(successful=True)

def main(args=None) -> None:
    rclpy.init(args=args)

    best_object_selector = BestObjectSelector()

    rclpy.spin(best_object_selector)

    best_object_selector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
