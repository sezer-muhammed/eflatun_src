import jetson.inference
import jetson.utils

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from rclpy.logging import LoggingSeverity
from eflatun_msgs.msg import TrackedObject, TrackedObjectArray

import os
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

        self.plane_detection_model = jetson.inference.detectNet(
            threshold=0.04,  # TODO test if its correct or hardcoded to 0.5
            argv=[
                '--model=' + self.params["model"]["detection_path"], '--labels=' + self.params["model"]["labels_path"],
                '--input-blob=images', '--output-cvg=scores', '--output-bbox=bboxes', '--log-level=warning'
            ])

        self.logitech_webcam = jetson.utils.videoSource(self.params[self.params["use_device"]]["device"],
                                                        argv=self.params["camera_args"])

        self.rtp_stream_output = jetson.utils.videoOutput(
            self.params["rtp_ip"], argv=["--headless", f"--bitrate={self.params['bitrate']['stream'] * 1024 * 1024}", "--log-level=silent"])

        self.video_file_output = jetson.utils.videoOutput(
            self.params[self.params["use_device"]]["save_path"],
            argv=["--headless", f"--bitrate={self.params['bitrate']['video'] * 1024 * 1024}", "--log-level=silent"])

        self.detections_publisher = self.create_publisher(TrackedObjectArray, "/webcam/detections", 1)

        self.create_subscription(TrackedObjectArray, self.params["visualization"]["topic"], self.stream_frame, 1)

        self.font = jetson.utils.cudaFont(size=32)

        self.create_timer(1 / 30, self.detect_objects)
        self.crop_roi = (int(self.params["model"]["detection_gap_ratio"] * self.params["video_width"]), 0,
                         self.params["video_width"] -
                         int(self.params["model"]["detection_gap_ratio"] * self.params["video_width"]),
                         self.params["video_height"])
        self.get_logger().info('Detection frame area: ({}, {}) - ({}, {})'.format(self.crop_roi[0], self.crop_roi[1],
                                                                                  self.crop_roi[2], self.crop_roi[3]))

    def get_frame(self):
        self.crop_roi = (int(self.params["model"]["detection_gap_ratio"] * self.params["video_width"]), 0,
                         self.params["video_width"] -
                         int(self.params["model"]["detection_gap_ratio"] * self.params["video_width"]),
                         self.params["video_height"])
        self.get_logger().debug(f"Detection model runs on given ROI: {self.crop_roi}")
        self.full_frame = self.logitech_webcam.Capture()
        self.detection_frame = jetson.utils.cudaAllocMapped(
            width=self.params["video_width"] -
            2 * int(self.params["model"]["detection_gap_ratio"] * self.params["video_width"]),
            height=self.params["video_height"],
            format=self.full_frame.format)
        jetson.utils.cudaCrop(self.full_frame, self.detection_frame, self.crop_roi)

    def detect_objects(self):
        self.get_frame()

        detections_msg = TrackedObjectArray()
        detections_msg.header.frame_id = "webcam"
        detections_msg.header.stamp = self.get_clock().now().to_msg()
        detections_msg.frame_seq = self.frame_counter
        self.frame_counter += 1

        self.detections = self.plane_detection_model.Detect(self.detection_frame, overlay="none")

        for single_detection in self.detections:
            tracked_detection = TrackedObject()
            tracked_detection.header = detections_msg.header
            tracked_detection.center_x = single_detection.Center[0] + int(
                self.params["model"]["detection_gap_ratio"] * self.params["video_width"])
            tracked_detection.center_y = single_detection.Center[1]
            tracked_detection.width = single_detection.Width
            tracked_detection.height = single_detection.Height

            detections_msg.detections.append(tracked_detection)

        self.detections_publisher.publish(detections_msg)

    def stream_frame(self, msg: TrackedObjectArray):

        # TODO Since this is a callback, there is delay but frame is current frame. So Bbox comes late. Fix it.

        jetson.utils.cudaDrawLine(self.full_frame, (int(self.params["visualization"]["margin"]["width_ratio"] * self.params["video_width"]), 
                                                int(self.params["visualization"]["margin"]["height_ratio"] * self.params["video_height"])),
                                        (int(self.params["visualization"]["margin"]["width_ratio"] * self.params["video_width"]), 
                                        int(self.params["video_height"] - self.params["visualization"]["margin"]["height_ratio"] * self.params["video_height"])), 
                                        tuple(self.params["visualization"]["colors"]["target_area"]), 
                                        int(self.params["visualization"]["thick"]))
        jetson.utils.cudaDrawLine(self.full_frame, (int(self.params["visualization"]["margin"]["width_ratio"] * self.params["video_width"]), 
                                                int(self.params["visualization"]["margin"]["height_ratio"] * self.params["video_height"])),
                                        (int(self.params["video_width"] - self.params["visualization"]["margin"]["width_ratio"] * self.params["video_width"]), 
                                        int(self.params["visualization"]["margin"]["height_ratio"] * self.params["video_height"])), 
                                        tuple(self.params["visualization"]["colors"]["target_area"]), 
                                        int(self.params["visualization"]["thick"]))
        jetson.utils.cudaDrawLine(self.full_frame, (int(self.params["video_width"] - self.params["visualization"]["margin"]["width_ratio"] * self.params["video_width"]), 
                                                int(self.params["visualization"]["margin"]["height_ratio"] * self.params["video_height"])),
                                        (int(self.params["video_width"] - self.params["visualization"]["margin"]["width_ratio"] * self.params["video_width"]), 
                                        int(self.params["video_height"] - self.params["visualization"]["margin"]["height_ratio"] * self.params["video_height"])), 
                                        tuple(self.params["visualization"]["colors"]["target_area"]), 
                                        int(self.params["visualization"]["thick"]))
        jetson.utils.cudaDrawLine(self.full_frame, (int(self.params["visualization"]["margin"]["width_ratio"] * self.params["video_width"]), 
                                                int(self.params["video_height"] - self.params["visualization"]["margin"]["height_ratio"] * self.params["video_height"])),
                                        (int(self.params["video_width"] - self.params["visualization"]["margin"]["width_ratio"] * self.params["video_width"]), 
                                        int(self.params["video_height"] - self.params["visualization"]["margin"]["height_ratio"] * self.params["video_height"])), 
                                        tuple(self.params["visualization"]["colors"]["target_area"]), 
                                        int(self.params["visualization"]["thick"]))


        for tracked_detection in msg.detections:

            x1 = tracked_detection.center_x - tracked_detection.width / 2
            x2 = tracked_detection.center_x + tracked_detection.width / 2
            y1 = tracked_detection.center_y - tracked_detection.height / 2
            y2 = tracked_detection.center_y + tracked_detection.height / 2

            object_id = f"id: {tracked_detection.unique_id}"

            self.font.OverlayText(self.full_frame, self.full_frame.width, self.full_frame.height, object_id, int(x1),
                                  int(y2), (255, 0, 0), (0, 0, 0))

            jetson.utils.cudaDrawLine(self.full_frame, (int(x1), int(y1)), (int(x2), int(y1)), tuple(self.params["visualization"]["colors"]["object"]), int(self.params["visualization"]["thick"]))
            jetson.utils.cudaDrawLine(self.full_frame, (int(x1), int(y1)), (int(x1), int(y2)), tuple(self.params["visualization"]["colors"]["object"]), int(self.params["visualization"]["thick"]))
            jetson.utils.cudaDrawLine(self.full_frame, (int(x1), int(y2)), (int(x2), int(y2)), tuple(self.params["visualization"]["colors"]["object"]), int(self.params["visualization"]["thick"]))
            jetson.utils.cudaDrawLine(self.full_frame, (int(x2), int(y1)), (int(x2), int(y2)), tuple(self.params["visualization"]["colors"]["object"]), int(self.params["visualization"]["thick"]))


        on_image_log = [
            f"{datetime.now()}",
            f"{self.params['model']['detection_path'].split('/')[-1]} @{round(self.plane_detection_model.GetNetworkFPS(), 2)} FPS"
        ]


        self.print_logs_on_image(on_image_log)

        self.rtp_stream_output.Render(self.full_frame)
        self.video_file_output.Render(self.full_frame)

    def print_logs_on_image(self, logs):

        for i, log in enumerate(logs):
            self.font.OverlayText(self.full_frame, self.full_frame.width, self.full_frame.height, log, 10, 32 * i + 10,
                                  (255, 0, 0), (0, 90, 0, 100))

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


def main(args=None):
    rclpy.init(args=args)
    node = JetsonDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
