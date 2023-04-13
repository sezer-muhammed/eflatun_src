import os
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from rclpy.logging import LoggingSeverity
from rclpy.qos import qos_profile_sensor_data
from eflatun_msgs.msg import TrackedObject, TrackedObjectArray
import cv2
import numpy as np
from datetime import datetime

today = datetime.now().strftime("%d_%m_%Y")
import json


class JetsonDetector(Node):

    def __init__(self):
        super().__init__("object_detector")

        # Generate the 'today' string
        today = datetime.now().strftime("%d_%m_%Y")  #TODO add this into save file name

        self.get_logger().info("Initializing JetsonDetector")

        self.declare_parameters(namespace='',
                                parameters=[
                                    ('log_level', Parameter.Type.STRING),
                                    ('use_device', Parameter.Type.STRING),
                                    ('video.device', Parameter.Type.STRING),
                                    ('video.save_path', Parameter.Type.STRING),
                                    ('video.round', Parameter.Type.INTEGER),
                                    ('cam.device', Parameter.Type.STRING),
                                    ('cam.save_path', Parameter.Type.STRING),
                                    ('cam.round', Parameter.Type.INTEGER),
                                    ('video_width', Parameter.Type.INTEGER),
                                    ('video_height', Parameter.Type.INTEGER),
                                    ('camera_args', Parameter.Type.STRING_ARRAY),
                                    ('visualization.colors.object', Parameter.Type.INTEGER_ARRAY),
                                    ('visualization.colors.target_area', Parameter.Type.INTEGER_ARRAY),
                                    ('visualization.colors.aqua', Parameter.Type.INTEGER_ARRAY),
                                    ('visualization.thick', Parameter.Type.INTEGER),
                                    ('visualization.font_size', Parameter.Type.INTEGER),
                                    ('visualization.margin.width_ratio', Parameter.Type.DOUBLE),
                                    ('visualization.margin.height_ratio', Parameter.Type.DOUBLE),
                                    ('visualization.topic', Parameter.Type.STRING),
                                    ('model.detection_path', Parameter.Type.STRING),
                                    ('model.labels_path', Parameter.Type.STRING),
                                    ('model.detection_gap_ratio', Parameter.Type.DOUBLE),
                                    ('rtp_ip', Parameter.Type.STRING),
                                    ('bitrate.stream', Parameter.Type.INTEGER),
                                    ('bitrate.video', Parameter.Type.INTEGER),
                                ])

        self.params = {
            'log_level': self.get_parameter('log_level').value,
            'use_device': self.get_parameter('use_device').value,
            'video': {
                'device': self.get_parameter('video.device').value,
                'save_path': self.get_parameter('video.save_path').value,
                'round': self.get_parameter('video.round').value
            },
            'cam': {
                'device': self.get_parameter('cam.device').value,
                'save_path': self.get_parameter('cam.save_path').value,
                'round': self.get_parameter('cam.round').value
            },
            'video_width': self.get_parameter('video_width').value,
            'video_height': self.get_parameter('video_height').value,
            'camera_args': self.get_parameter('camera_args').value,
            'visualization': {
                'colors': {
                    "object": self.get_parameter('visualization.colors.object').value,
                    "target_area": self.get_parameter('visualization.colors.target_area').value,
                    'aqua': self.get_parameter('visualization.colors.aqua').value
                },
                'thick': self.get_parameter('visualization.thick').value,
                'font_size': self.get_parameter('visualization.font_size').value,
                'margin': {
                    'width_ratio': self.get_parameter('visualization.margin.width_ratio').value,
                    'height_ratio': self.get_parameter('visualization.margin.height_ratio').value
                },
                'topic': self.get_parameter('visualization.topic').value
            },
            'model': {
                'detection_path': self.get_parameter('model.detection_path').value,
                'labels_path': self.get_parameter('model.labels_path').value,
                'detection_gap_ratio': self.get_parameter('model.detection_gap_ratio').value
            },
            'rtp_ip': self.get_parameter('rtp_ip').value,
            'bitrate': {
                'stream': self.get_parameter('bitrate.stream').value,
                'video': self.get_parameter('bitrate.video').value
            }
        }

        log_level_mapping = {
            'debug': LoggingSeverity.DEBUG,
            'info': LoggingSeverity.INFO,
            'warn': LoggingSeverity.WARN,
            'error': LoggingSeverity.ERROR,
            'fatal': LoggingSeverity.FATAL,
        }
        log_level = log_level_mapping.get(self.params["log_level"], LoggingSeverity.INFO)
        self.get_logger().set_level(log_level)

        self.get_logger().info(json.dumps(self.params, sort_keys=True, indent=4))

        # Subscribe to parameter changes
        self.add_on_set_parameters_callback(self.on_parameter_change)

        self.frame_counter = 0

        self.detections_publisher = self.create_publisher(TrackedObjectArray, "/webcam/detections", qos_profile_sensor_data)

        self.create_timer(1 / 30, self.detect_objects)
        self.crop_roi = (int(self.params["model"]["detection_gap_ratio"] * self.params["video_width"]), 0,
                         self.params["video_width"] -
                         int(self.params["model"]["detection_gap_ratio"] * self.params["video_width"]),
                         self.params["video_height"])
        self.get_logger().info('Detection frame area: ({}, {}) - ({}, {})'.format(self.crop_roi[0], self.crop_roi[1],
                                                                                  self.crop_roi[2], self.crop_roi[3]))
        self.mouse_position_x = 0
        self.mouse_position_y = 0
        cv2.namedWindow("frame")
        cv2.setMouseCallback("frame", self.mouse_callback)

    def mouse_callback(self, event, x, y, flags, param):

        if event == cv2.EVENT_MOUSEMOVE:
            self.mouse_position_x = x * 1920.0 / 640.0
            self.mouse_position_y = y * 1080.0 / 640.0

    def detect_objects(self) -> None:
        # Get a frame from the webcam.
        frame = np.ones((640, 640, 3), np.uint8) * 255

        cv2.imshow("frame", frame)
        cv2.waitKey(1)

        detections_msg = TrackedObjectArray()
        detections_msg.header.frame_id = "webcam"
        detections_msg.header.stamp = self.get_clock().now().to_msg()
        detections_msg.frame_seq = self.frame_counter
        self.frame_counter += 1

        # Populate the detections message.
        tracked_detection = TrackedObject()

        tracked_detection.header.frame_id = "webcam"
        tracked_detection.header.stamp = self.get_clock().now().to_msg()
        tracked_detection.center_x = float(self.mouse_position_x)
        tracked_detection.center_y = float(self.mouse_position_y)
        tracked_detection.width = 150.0
        tracked_detection.height = 150.0

        detections_msg.detections.append(tracked_detection)

        # Publish the detections message.
        self.detections_publisher.publish(detections_msg)

    def on_parameter_change(self, params):
        # Update the parameter dictionary with the new values
        for param in params:
            try:
                # Get the key and value
                param_name = param.name
                param_value = param.value

                # Split the key into a list of keys
                param_name = param_name.split(".")

                # Get the dictionary to update from the list of keys
                params_temp = self.params
                for key in param_name[:-1]:
                    params_temp = params_temp[key]

                # Update the dictionary
                params_temp[param_name[-1]] = param_value
                self.get_logger().info('Parameter {} updated to {}'.format(param_name, param_value))
            except:
                self.get_logger().warning('Parameter {} cannot updated to {}'.format(param_name, param_value))
                return SetParametersResult(successful=False)
        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)
    node = JetsonDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
