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
    uint32 missing_age

    uint32 center_x
    uint32 center_y

    uint32 width
    uint32 height
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from typing import List, Tuple, Dict
import numpy as np
from scipy.spatial.distance import cdist
from eflatun_msgs.msg import TrackedObject, TrackedObjectArray
from eflatun.Object import Object


class ObjectTracker:

    def __init__(self, max_missing_frames: int = 4) -> None:
        self.objects: Dict[int, Object] = {}
        self.next_unique_id = 1
        self.max_missing_frames = max_missing_frames
        self.distance_calculator = DistanceCalculator()

    def track_objects(self, detected_object_msgs: List[TrackedObject]) -> List[TrackedObject]:
        if not self.objects:
            print("Self Objects Boş, Her Gelen Ekleniyor.")
            for obj in detected_object_msgs:
                new_object = Object(obj)
                new_object.unique_id = self.next_unique_id
                new_object.missing_age = 1
                self.objects[self.next_unique_id] = new_object
                self.next_unique_id += 1

        else:
            for obj in detected_object_msgs:
                new_object = Object(obj)
                closest_object_id, distance = self.distance_calculator.find_closest_object(
                    new_object, self.objects.values())

                if self.objects[closest_object_id] is not None and distance < 100:
                    self.objects[closest_object_id].update(new_object.center_x, new_object.center_y, new_object.width,
                                                           new_object.height)
                else:
                    new_object.unique_id = self.next_unique_id
                    self.objects[self.next_unique_id] = new_object
                    self.next_unique_id += 1

            # Predict the location of missing objects and increase their missing age

            for obj in self.objects.copy().values():
                if obj.missing_age >= 1 and obj.age > 8:
                    obj.predict()

                if obj.missing_age > self.max_missing_frames:
                    del self.objects[obj.unique_id]
                    continue
                    
                obj.missing_age += 1
                # If the object is still missing after max_missing_frames, remove it from the objects dictionary


        tracked_objects = [
            obj.to_ros_message() if obj.missing_age <= 1 else obj.from_prediction_to_ros_message()
            for obj in self.objects.values()
        ]
        return tracked_objects


class DistanceCalculator:

    def __init__(self) -> None:
        pass

    def find_closest_object(self, new_object: Object, tracked_objects: List[Object]) -> Tuple[int, float]:
        """
        Find the closest object to the new_object among the tracked_objects using Euclidean distance.

        Args:
        new_object (Object): The new object to find the closest tracked object for.
        tracked_objects (List[Object]): A list of tracked objects.

        Returns:
        Tuple[int, float]: The unique ID of the closest tracked object and the Euclidean distance.
        """
        tracked_objects = list(tracked_objects)
        new_object_coords = np.array([new_object.center_x, new_object.center_y])
        tracked_objects_coords = np.array([[obj.center_x, obj.center_y] for obj in tracked_objects])
        distances = cdist(new_object_coords.reshape(1, -1), tracked_objects_coords, metric='euclidean')
        min_distance_index = np.argmin(distances)
        closest_object_id =tracked_objects[min_distance_index].unique_id
        min_distance = distances[0, min_distance_index]

        return closest_object_id, min_distance

    def __repr__(self) -> str:
        return f"DistanceCalculator(metric='euclidean')"


class ObjectTrackingNode(Node):

    def __init__(self):
        super().__init__('tracker')
        qos_profile = QoSProfile(depth=2)
        self.tracked_objects_publisher = self.create_publisher(TrackedObjectArray, '/tracker/tracked_objects', qos_profile)
        self.detected_objects_subscription = self.create_subscription(TrackedObjectArray, '/webcam/detections',
                                                                      self.detected_objects_callback, qos_profile)
        self.object_tracker = ObjectTracker()

    def detected_objects_callback(self, msg: TrackedObjectArray):
        detected_objects = msg.detections
        tracked_objects = self.object_tracker.track_objects(detected_objects)

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.detections = tracked_objects
        self.tracked_objects_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    object_tracking_node = ObjectTrackingNode()
    rclpy.spin(object_tracking_node)
    object_tracking_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
