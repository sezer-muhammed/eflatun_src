# eflatun

[![CircleCI](https://dl.circleci.com/status-badge/img/gh/sezer-muhammed/eflatun_src/tree/master.svg?style=svg)](https://dl.circleci.com/status-badge/redirect/gh/sezer-muhammed/eflatun_src/tree/master)

## Overview

Eflatun is a ROS2 package developed for detecting and tracking fixed-wing airplanes. It has been specifically designed for the Teknofest 2023 event. The package is intended for robotic and computer vision applications and leverages a customized version of the Jetson Inference library for high-performance object detection, achieving up to 85 FPS with YOLOv8n.

The package consists of several nodes that work together to accomplish the task of detection and tracking:

- `object_detector`: Detects objects and publishes information about them.
- `object_tracker`: Assigns unique IDs to detected objects for tracking purposes.
- `best_object_selector`: Selects the best object to track based on certain criteria.
- `vehicle_status_gui`: Visualizes the tracked object and provides a user interface for controlling the ROS2 nodes.

The eflatun package is actively maintained and supported, with plans for future updates and improvements.


## Installation

### Prerequisites

### Building the Package

## Nodes

### object_tracker

The `object_tracker` node is responsible for tracking detected fixed-wing airplanes using the custom messages `TrackedObject` and `TrackedObjectArray`. It receives detected objects from the `detector` node and maintains a list of tracked objects with unique IDs. The node updates the position of the tracked objects and predicts their location if they are missing for a certain number of frames. It also removes objects that have been missing for more than a specified number of frames. The node publishes the tracked objects as `TrackedObjectArray` messages.

The node is implemented in the `ObjectTracker` and `ObjectTrackingNode` classes. The `ObjectTracker` class handles object tracking, while the `ObjectTrackingNode` class is responsible for ROS2-related functionality like parameter handling, subscriptions, and publishing.

Parameters for the `object_tracker` node include:

- `log_level`: The logging level for the node (e.g., "info", "debug", "warn", etc.).
- `max_missing_frames`: The maximum number of frames an object can be missing before it is removed from tracking.
- `distance_threshold`: The maximum distance between a detected object and a tracked object for them to be considered the same object.
- `min_age_to_predict`: The minimum age of an object before its location can be predicted if it is missing.

The node subscribes to the `/webcam/detections` topic to receive detected objects and publishes the tracked objects on the `/tracker/tracked_objects` topic.


### object_detector

This package provides a ROS2 node for object detection using YOLOv8n neural network model on NVIDIA Jetson devices.

#### Parameters

The node reads the following parameters from the ROS2 parameter server:

- `log_level`: Log level for the node. Possible values are `"debug"`, `"info"`, `"warn"`, `"error"`, and `"fatal"`. Default value is `"info"`.
- `use_device`: Specifies which device to use. Possible values are `"video"` and `"cam"`. Default value is `"video"`.
- `video`: Parameters for video device. If `use_device` is set to `"video"`, these parameters are used. Contains the following fields:
    - `device`: Path to video file to be used as input.
    - `save_path`: Path to save the output video.
    - `round`: Number of times the video is to be looped. Default value is `1`.
- `cam`: Parameters for camera device. If `use_device` is set to `"cam"`, these parameters are used. Contains the following fields:
    - `device`: Path to camera device.
    - `save_path`: Path to save the output video.
    - `round`: Number of times the video is to be looped. Default value is `1`.
- `video_width`: Width of the video frame. Default value is `1920`.
- `video_height`: Height of the video frame. Default value is `1080`.
- `camera_args`: Arguments to be passed to the camera device. Default value is `["--input-width=1920", "--input-height=1080", "--log-level=silent"]`.
- `visualization`: Parameters for visualization of detected objects. Contains the following fields:
    - `colors`: Dictionary containing color values for different elements. Contains the following fields:
        - `object`: Color for object bounding box.
        - `target_area`: Color for target area bounding box.
        - `aqua`: Color for text overlay.
    - `thick`: Thickness of bounding box lines.
    - `font_size`: Font size for text overlay.
    - `margin`: Margin values for target area bounding box. Contains the following fields:
        - `width_ratio`: Width ratio of margin. Default value is `0.25`.
        - `height_ratio`: Height ratio of margin. Default value is `0.1`.
    - `topic`: ROS2 topic to subscribe for object detections. Default value is `"/tracker/tracked_objects"`.
- `model`: Parameters for object detection model. Contains the following fields:
    - `detection_path`: Path to object detection model file.
    - `labels_path`: Path to object detection model labels file.
    - `detection_gap_ratio`: Ratio of the frame to be used for object detection. Default value is `0.1`.
    - `confidence`: Minimum confidence threshold for object detection. Default value is `0.3`.
- `rtp_ip`: IP address of the destination for RTP streaming. Default value is `"rtp://192.168.1.4:1234"`.
- `bitrate`: Bitrate values for stream and output videos. Contains the following fields:
    - `stream`: Bitrate for RTP streaming in MBPS. Default value is `15`.
    - `video`: Bitrate for output video in MBPS. Default value is `5`.

### object

### Best Object Selector Node

The Best Object Selector node is a ROS2 node that subscribes to a topic containing a list of tracked objects and selects the best object based on some criteria. The selected object is published to a different topic.

#### Workflow

The node subscribes to the `/tracker/tracked_objects` topic, which contains a list of tracked objects. For each object, the node calculates a score based on its width, height, age, and distance to a fixed center point. The object with the highest score is selected and published to the `/tracker/best_object` topic.

#### Parameters

The behavior of the node can be configured using the following ROS2 parameters:
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| log_level | string | info | Log level of the node |
| frame_center.x | integer | 960 | x coordinate of the center of the frame |
| frame_center.y | integer | 580 | y coordinate of the center of the frame |
| x_range.min | integer | 480 | minimum x coordinate of the frame |
| x_range.max | integer | 1440 | maximum x coordinate of the frame |
| y_range.min | integer | 108 | minimum y coordinate of the frame |
| y_range.max | integer | 972 | maximum y coordinate of the frame |
| min_width | integer | 80 | minimum width of the object to be considered |
| min_height | integer | 40 | minimum height of the object to be considered |
| score_variables.width_weight | double | 0.5 | weight of the object's width in the scoring formula |
| score_variables.height_weight | double | 0.5 | weight of the object's height in the scoring formula |
| score_variables.age_weight | double | 1.0 | weight of the object's age in the scoring formula |
| score_variables.dist_to_center_weight | double | 0.2 | weight of the distance to the frame center in the scoring formula |


### vehicle_status_gui

## Custom Message Types

### TrackedObject

### TrackedObjectArray

## License
