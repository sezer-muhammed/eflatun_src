"""
    # TrackedObjectArray.msg

    std_msgs/Header header

    uint64 frame_seq
    eflatun_msgs/TrackedObject[] detections

=================================================
    
    TrackedObject.msg

    std_msgs/Header header

    uint32 unique_id
    uint32 age

    uint32 center_x
    uint32 center_y

    uint32 width
    uint32 height
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from typing import List, Tuple
import numpy as np
from scipy.spatial.distance import cdist
from filterpy.kalman import KalmanFilter
from eflatun_msgs.msg import TrackedObject, TrackedObjectArray
from eflatun.Object import Object


class ObjectTracker:

    def __init__(self) -> None:
        pass


class DistanceCalculator:

    def __init__(self) -> None:
        pass


class ObjectTrackingNode(Node):

    def __init__(self):
        super().__init__('object_tracking_node')
        qos_profile = QoSProfile(depth=2)
        self.tracked_objects_publisher = self.create_publisher(TrackedObjectArray, 'tracked_objects', qos_profile)
        self.detected_objects_subscription = self.create_subscription(TrackedObjectArray, 'detected_objects',
                                                                      self.detected_objects_callback, qos_profile)
        self.object_tracker = ObjectTracker()

    def detected_objects_callback(self, msg: TrackedObjectArray):
        detected_objects = msg.objects
        tracked_objects = self.object_tracker.track_objects(detected_objects)

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.objects = tracked_objects
        self.tracked_objects_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    object_tracking_node = ObjectTrackingNode()
    rclpy.spin(object_tracking_node)
    object_tracking_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
