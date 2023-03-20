import jetson.inference
import jetson.utils

import rclpy
from rclpy.node import Node
from eflatun_msgs.msg import TrackedObject, TrackedObjectArray

import os
from datetime import datetime

today = datetime.now().strftime("%d_%m_%Y")

if True:
    VIDEO_DEVICE = "file:///home/iha/Desktop/eflatun_ros/fake_video.mp4"
    VIDEO_SAVE_FILE = f"fake/FAKE_{1}_Eflatun_IHA_{today}.mp4"

    NAMESPACE = "fake_camera"
    NODENAME = "all_detections"
    DETECTION_PUBLISHERNAME = "fake_detections"

    VIDEO_WIDTH = 1920
    VIDEO_HEIGHT = 1080

    CAMERA_ARGS = [f"--input-width={VIDEO_WIDTH}", f"--input-height={VIDEO_HEIGHT}"]

else:
    VIDEO_DEVICE = "v4l2:///dev/video0"
    VIDEO_SAVE_FILE = f"{args.round}_Eflatun_IHA_{today}.mp4"

    NAMESPACE = "camera"
    NODENAME = "all_detections"
    DETECTION_PUBLISHERNAME = "detections"

    VIDEO_WIDTH = 1920
    VIDEO_HEIGHT = 1080

    CAMERA_ARGS = [f"--input-width={VIDEO_WIDTH}", f"--input-height={VIDEO_HEIGHT}"]

COLOR_RED = (255, 0, 0, 255)
COLOR_GREEN = (20, 255, 50, 255)
THICK = 2

MARGIN_WIDTH = VIDEO_WIDTH // 4
MARGIN_HEIGHT = VIDEO_HEIGHT // 10

DETECTION_GAP = VIDEO_WIDTH * 2 // 20

RTP_IP = "rtp://192.168.1.2:1234"
DETECTION_MODEL_PATH = os.path.abspath(args.model)
LABELS_PATH = "/home/iha/Desktop/eflatun_ros/model/labels.txt"

BITRATE_STREAM = 1  #MBPS
BITRATE_VIDEO = 1  #MBPS


class JetsonDetector(Node):

    def __init__(self):
        super().__init__(NODENAME, namespace=NAMESPACE)

        self.get_logger().info("Initializing...")

        self.frame_counter = 0

        self.plane_detection_model = jetson.inference.detectNet(
            threshold=0.4,  # TODO test if its correct or hardcoded to 0.5
            argv=[
                '--model=' + DETECTION_MODEL_PATH, '--labels=' + LABELS_PATH, '--input-blob=images',
                '--output-cvg=scores', '--output-bbox=bboxes'
            ])

        self.logitech_webcam = jetson.utils.videoSource(VIDEO_DEVICE, argv=CAMERA_ARGS)

        self.rtp_stream_output = jetson.utils.videoOutput(
            RTP_IP, argv=["--headless", f"--bitrate={BITRATE_STREAM * 1024 * 1024}"])

        self.video_file_output = jetson.utils.videoOutput(
            VIDEO_SAVE_FILE, argv=["--headless", f"--bitrate={BITRATE_VIDEO * 1024 * 1024}"])

        self.detections_publisher = self.create_publisher(TrackedObjectArray, "/webcam/detections", 1)

        self.create_subscription(TrackedObjectArray, "/tracker/detections_tracked", self.stream_frame, 1)

        self.font = jetson.utils.cudaFont(size=32)

        self.create_timer(1 / 30, self.detect_objects)
        self.crop_roi = (DETECTION_GAP, 0, VIDEO_WIDTH - DETECTION_GAP, VIDEO_HEIGHT)
        self.get_logger().info("Detection and stream node has been created")

    def get_frame(self):
        self.full_frame = self.logitech_webcam.Capture()
        self.detection_frame = jetson.utils.cudaAllocMapped(width=VIDEO_WIDTH - 2 * DETECTION_GAP,
                                                            height=VIDEO_HEIGHT,
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
            tracked_detection.center_x = int(single_detection.Center[0] + DETECTION_GAP)
            tracked_detection.center_y = int(single_detection.Center[1])
            tracked_detection.width = int(single_detection.Width)
            tracked_detection.height = int(single_detection.Height)

            detections_msg.detections.append(tracked_detection)

        self.detections_publisher.publish(detections_msg)

    def stream_frame(self, msg):

        # TODO Since this is a callback, there is delay but frame is current frame. So Bbox comes late. Fix it.

        jetson.utils.cudaDrawLine(self.full_frame, (MARGIN_WIDTH, MARGIN_HEIGHT),
                                  (MARGIN_WIDTH, VIDEO_HEIGHT - MARGIN_HEIGHT), COLOR_GREEN, THICK)
        jetson.utils.cudaDrawLine(self.full_frame, (MARGIN_WIDTH, MARGIN_HEIGHT),
                                  (VIDEO_WIDTH - MARGIN_WIDTH, MARGIN_HEIGHT), COLOR_GREEN, THICK)
        jetson.utils.cudaDrawLine(self.full_frame, (VIDEO_WIDTH - MARGIN_WIDTH, MARGIN_HEIGHT),
                                  (VIDEO_WIDTH - MARGIN_WIDTH, VIDEO_HEIGHT - MARGIN_HEIGHT), COLOR_GREEN, THICK)
        jetson.utils.cudaDrawLine(self.full_frame, (MARGIN_WIDTH, VIDEO_HEIGHT - MARGIN_HEIGHT),
                                  (VIDEO_WIDTH - MARGIN_WIDTH, VIDEO_HEIGHT - MARGIN_HEIGHT), COLOR_GREEN, THICK)

        for tracktion in msg.detections:

            x1 = tracktion.bbox.center.x - tracktion.bbox.size_x / 2
            x2 = tracktion.bbox.center.x + tracktion.bbox.size_x / 2
            y1 = tracktion.bbox.center.y - tracktion.bbox.size_y / 2
            y2 = tracktion.bbox.center.y + tracktion.bbox.size_y / 2

            object_id = "id: " + tracktion.results[0].id

            self.font.OverlayText(self.full_frame, self.full_frame.width, self.full_frame.height, object_id, int(x1),
                                  int(y2), (255, 0, 0), (0, 0, 0))

            jetson.utils.cudaDrawLine(self.full_frame, (x1, y1), (x2, y1), COLOR_RED, THICK)
            jetson.utils.cudaDrawLine(self.full_frame, (x1, y1), (x1, y2), COLOR_RED, THICK)
            jetson.utils.cudaDrawLine(self.full_frame, (x1, y2), (x2, y2), COLOR_RED, THICK)
            jetson.utils.cudaDrawLine(self.full_frame, (x2, y1), (x2, y2), COLOR_RED, THICK)

        on_image_log = [
            f"{datetime.now()}",
            f"{DETECTION_MODEL_PATH.split('/')[-1]} @{round(self.plane_detection_model.GetNetworkFPS(), 2)} FPS"
        ]

        self.print_logs_on_image(on_image_log)

        self.rtp_stream_output.Render(self.full_frame)
        self.video_file_output.Render(self.full_frame)

    def print_logs_on_image(self, logs):

        for i, log in enumerate(logs):
            self.font.OverlayText(self.full_frame, self.full_frame.width, self.full_frame.height, log, 10, 32 * i + 10,
                                  (255, 0, 0), (0, 90, 0, 100))


def main(args=None):
    rclpy.init(args=args)
    node = JetsonDetector()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
