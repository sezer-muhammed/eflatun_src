Build status: [![CircleCI](https://dl.circleci.com/status-badge/img/gh/sezer-muhammed/eflatun_src/tree/master.svg?style=svg)](https://dl.circleci.com/status-badge/redirect/gh/sezer-muhammed/eflatun_src/tree/master)

# Contact Information

If you have any questions or need support, feel free to reach out to our team members:

- **Şevval Belkıs Dikkaya**: Head of the team, working on software, electronics, and mechanics. [LinkedIn](https://www.linkedin.com/in/sbdikkaya/)
- **Muhammed Sezer**: Responsible for electronics and software, and the maintainer of this package. [LinkedIn](https://www.linkedin.com/in/muhammed-sezer-160428208/)
- **Metehan İçöz**: Responsible for mechanics. [LinkedIn](https://www.linkedin.com/in/metehan-içöz-735bab206/)
- **Zeynep Keleş**: Working on software, electronics, and mechanics. [LinkedIn](https://www.linkedin.com/in/zeynep-bilge-keleş-7ba917255/)

Feel free to connect with us on LinkedIn and mention any specific questions or issues related to the project. We'll be happy to help!

# Object Detection and Tracking ROS2 Package

## Table of Contents

1. [Introduction](#introduction)
2. [Dependencies](#dependencies)
3. [Installation](#installation)
4. [Usage](#usage)
5. [Node Descriptions](#node-descriptions)
    - [object_detector](#object_detector)
        - [Published Topics](#published-topics)
        - [Subscribed Topics](#subscribed-topics)
    - [object_tracker](#object_tracker)
        - [Published Topics](#published-topics-1)
        - [Subscribed Topics](#subscribed-topics-1)
    - [best_object_selector](#best_object_selector)
        - [Published Topics](#published-topics-2)
        - [Subscribed Topics](#subscribed-topics-2)
    - [vehicle_status](#vehicle_status)
        - [Published Topics](#published-topics-3)
        - [Subscribed Topics](#subscribed-topics-3)
6. [Parameters](#parameters)
7. [Messages](#messages)
8. [Code Examples](#code-examples)
9. [Troubleshooting](#troubleshooting)
10. [Contributing](#contributing)
11. [License](#license)
12. [Hardware Used](#hardware-used)
13. [Sources](#sources)

## Introduction

The introduction section should provide a brief overview of the main features, capabilities, and use cases of the package. Explain the purpose of the package and what it aims to achieve.

Eflatun is a package that contains a comprehensive set of tools and algorithms for implementing and testing machine learning models. This project is aimed at providing developers with a modular and extensible software framework for developing autonomous systems, such as robots, drones, and other intelligent machines. The package includes nodes that track and detect an object and select the best out of them. It also includes instructions for how to install and use the framework, as well as examples of how to develop various autonomous systems using the eflatun_src framework..

The package is capable of data preprocessing, feature engineering, and model evaluation. It also supports parallel computing, which enables users to take advantage of multi-core CPUs for faster processing of large datasets.

The package can be used for a variety of domains such as finance, healthcare, marketing, and research purposes. Its features and capabilities make it a valuable tool for anyone interested in machine learning and data science. The use cases of Eflatun include predictive modeling, anomaly detection, image and speech recognition, natural language processing (NLP), and recommender systems.

## Dependencies

In the Dependencies section, list and explain all external dependencies required for the package to work, including specific versions if necessary. Add instructions for installing these dependencies.

## Installation

The Installation section should provide step-by-step installation instructions for the package. Include information on installing the package from source or using a package manager, if applicable.

## Usage

The Usage section should include detailed examples demonstrating how to use the package. Provide code snippets, terminal commands, or configuration files to help users understand the package's functionality better.

## Node Descriptions

### object_detector

The `object_detector` node is responsible for detecting objects in a video stream. It processes the video frames and identifies objects based on the provided model. The node does not subscribe to any topics.

#### Published Topics

- `/webcam/detections` (eflatun_msgs/TrackedObjectArray): A list of detected objects in the video stream, including their positions, sizes, and class IDs. This topic is published by the object detection node and consumed by the `object_tracker` node for further processing and tracking.

#### Subscribed Topics

The `object_detector` node does not subscribe to any topics.

### object_tracker

The `object_tracker` node is responsible for tracking detected objects in a video stream. It subscribes to the `/webcam/detections` topic, receives detected objects, and updates their tracking information. The tracking information is then published to the `/tracker/tracked_objects` topic.

#### Published Topics

- `/tracker/tracked_objects` (eflatun_msgs/TrackedObjectArray): A list of tracked objects, containing their unique IDs, positions, sizes, and ages.

#### Subscribed Topics

- `/webcam/detections` (eflatun_msgs/TrackedObjectArray): A list of detected objects in the video stream, published by the object detection node.


### best_object_selector

The `best_object_selector` is a ROS node responsible for selecting the best object from a list of tracked objects. It subscribes to the `/tracker/tracked_objects` topic, calculates a score for each object based on its properties (width, height, age, and distance to center), and publishes the best object to the `/tracker/best_object` topic. It also takes into account the minimum size of an object and if it lies within the specified range.

#### Published Topics

- `/tracker/best_object` (eflatun_msgs/TrackedObject): The best object selected based on the calculated score.

#### Subscribed Topics

- `/tracker/tracked_objects` (eflatun_msgs/TrackedObjectArray): A list of tracked objects published by the object tracker.


### vehicle_status

This is a Python script for a simple GUI that subscribes to multiple MAVROS topics using ROS2 and displays the received data in a table.

The `MavrosSubscriber` class is a ROS2 node that subscribes to the provided topics using the `create_subscription` method from the `Node` class. When a message is received on a subscribed topic, the corresponding callback function is called to update the GUI.

The `MavrosGUI` class is a PyQt5 widget that displays a table with the subscribed topics and their values. The `update_table` method updates the table with the received message.                     

#### Published Topics

The `vehicle_status` node doesn't publish to any topics.

#### Subscribed Topics

- `/diagnostics` (DiagnosticArray): provides diagnostic information about the system.
- `/mavros/battery` (BatteryState): provides information about the battery level.
- `/mavros/mavros/data` (Imu): provides data from the IMU (Inertial Measurement Unit).
- `/mavros/mavros/data_raw` (Imu): provides raw data from the IMU.
- `/mavros/mavros/diff_pressure` (FluidPressure): provides differential pressure data.
- `/mavros/mavros/in` (RCIn): provides information about the input channels.
- `/mavros/mavros/mag` (MagneticField): provides magnetometer data.
- `/mavros/mavros/out` (RCOut): provides information about the output channels.
- `/mavros/mavros/output` (NavControllerOutput): provides information about the navigation controller output.
- `/mavros/mavros/raw/fix` (NavSatFix): provides raw GPS data.
- `/mavros/mavros/raw/gps_vel` (TwistStamped): provides raw GPS velocity data.
- `/mavros/mavros/raw/satellites` (UInt32): provides information about GPS satellites.


## Parameters

### object_detector

| Parameter                    | Default Value                                         | Description                                                   |
|------------------------------|-------------------------------------------------------|---------------------------------------------------------------|
| log_level                    | debug                                                 | The logging level (debug, info, warn, error, fatal)          |
| use_device                   | video                                                 | Choose between "video" and "cam" for input source            |
| video.device                 | file:///path/to/video.mp4 | The video file used as input                                  |
| video.save_path              | /path/to/save/file.mp4 | The output video file path                                    |
| video.round                  | 1                                                     | Round parameter for video processing                          |
| cam.device                   | v4l2:///dev/video0                                    | The camera device used as input                               |
| cam.save_path                | /path/to/save/file.mp4                             | The output video file path for camera input                   |
| cam.round                    | 1                                                     | Round parameter for camera processing                         |
| video_width                  | 1920                                                  | Video width                                                   |
| video_height                 | 1080                                                  | Video height                                                  |
| camera_args                  | ["--input-width=1920", "--input-height=1080", "--log-level=silent"] | Camera arguments                                              |
| visualization.colors.object  | [255, 0, 0, 255]                                      | Color for object bounding boxes                               |
| visualization.colors.target_area | [20, 255, 50, 255]                                | Color for the target area                                     |
| visualization.colors.aqua    | [0, 255, 255, 255]                                    | Color for additional visualization elements |
| visualization.thick | 2 | Thickness for drawing lines and boxes |
| visualization.font_size | 32 | Font size for text |
| visualization.margin.width_ratio | 0.25 | Width margin ratio for visualization elements |
| visualization.margin.height_ratio | 0.1 | Height margin ratio for visualization elements |
| visualization.topic | /tracker/tracked_objects | Topic for the TrackedObjects messages |
| model.detection_path | /path/to/model.onnx | Path to the object detection model file |
| model.labels_path | /path/to/labels.txt | Path to the labels file |
| model.detection_gap_ratio | 0.1 | The gap ratio for object detection |
| model.confidence | 0.3 | Confidence threshold for object detection |
| rtp_ip | rtp://192.168.1.4:1234 | RTP IP for video streaming |
| bitrate.stream | 15 | Streaming bitrate in MBPS |
| bitrate.video | 5 | Video bitrate in MBPS |


### object_tracker

| Parameter                    | Default Value                                         | Description                                                   |
|------------------------------|-------------------------------------------------------|---------------------------------------------------------------|
| log_level                    | debug                                                 | The logging level (debug, info, warn, error, fatal)          |
| max_missing_frames           | 30                                                    | Maximum number of missing frames before an object is removed |
| distance_threshold           | 200                                                   | Distance threshold for object association                     |
| min_age_to_predict           | 5                                                     | Minimum age of an object for prediction                       |

### vehicle_status_gui

| Parameter                    | Default Value                                         | Description                                                   |
|------------------------------|-------------------------------------------------------|---------------------------------------------------------------|
| log_level                    | debug                                                 | The logging level (debug, info, warn, error, fatal)          |
| topics                       | ["empty",]                                            | Topics to visualize TrackedDetectionArray or TrackedDetection |
| frame_rate                   | 20                                                    | Frame rate for visualization                                  |


### best_object_selector

| Parameter              | Default Value | Description                                                   |
|------------------------|---------------|---------------------------------------------------------------|
| log_level              | debug         | The logging level (debug, info, warn, error, fatal)          |
| frame_center.x         | 960           | The x-coordinate of the frame center                          |
| frame_center.y         | 580           | The y-coordinate of the frame center                          |
| x_range.min            | 480           | The minimum x-coordinate for the object selection area        |
| x_range.max            | 1440          | The maximum x-coordinate for the object selection area        |
| y_range.min            | 108           | The minimum y-coordinate for the object selection area        |
| y_range.max            | 972           | The maximum y-coordinate for the object selection area        |
| min_width              | 80            | The minimum width of an object to be considered for selection |
| min_height             | 40            | The minimum height of an object to be considered for selection|
| score_variables.width_weight  | 0.5     | The weight given to the object's width in the score calculation |
| score_variables.height_weight | 0.5     | The weight given to the object's height in the score calculation |
| score_variables.age_weight    | 1.0     | The weight given to the object's age in the score calculation |
| score_variables.dist_to_center_weight | 0.2 | The weight given to the object's distance to the center in the score calculation |






## Custom Messages

### TrackedObject.msg

| Field       | Type               | Description                                          |
|-------------|--------------------|------------------------------------------------------|
| header      | std_msgs/Header    | Standard message header with timestamp and frame_id |
| unique_id   | uint32             | A unique identifier for the tracked object          |
| age         | uint32             | The age of the tracked object in frames             |
| missing_age | uint32             | The number of consecutive frames the object has been missing |
| center_x    | float32            | The x-coordinate of the object's center             |
| center_y    | float32            | The y-coordinate of the object's center             |
| width       | float32            | The width of the object's bounding box              |
| height      | float32            | The height of the object's bounding box             |

### TrackedObjectArray.msg

| Field      | Type                         | Description                                          |
|------------|------------------------------|------------------------------------------------------|
| header     | std_msgs/Header              | Standard message header with timestamp and frame_id |
| frame_seq  | uint64                       | Frame sequence number                                |
| detections | eflatun_msgs/TrackedObject[] | An array of tracked objects                          |


## Code Examples

The Code Examples section should provide examples demonstrating how to use the package in various scenarios. Include code snippets or configuration files to give users a clear understanding of how to implement the package in their projects.

## Contributing

We welcome and appreciate contributions from the community! If you're interested in contributing to this project, please follow these guidelines to ensure a smooth collaboration process:

1. **Reporting Issues**: If you encounter any issues or have suggestions for improvements, please create an issue on the project's GitHub repository. When reporting an issue, provide a clear and concise description of the problem, steps to reproduce it, and any additional information (e.g., screenshots, logs) that might be helpful in diagnosing and resolving the issue.

2. **Submitting Pull Requests**: If you want to contribute code, start by forking the repository and creating a new branch for your changes. Make sure your code follows the project's coding style and conventions. Before submitting a pull request, ensure that your changes do not introduce any new errors or warnings, and that all tests pass. In your pull request, provide a brief summary of the changes, as well as any relevant issue numbers.

3. **Participating in Development Discussions**: If you have ideas for new features or would like to provide feedback on the project's direction, join the development discussions by reaching out to the team members via LinkedIn or other communication channels mentioned in the Contact Information section.

4. **Documentation**: If you notice any errors or omissions in the project's documentation, please submit a pull request with the necessary changes, or create an issue to bring it to our attention.

5. **Testing**: Help us improve the quality and stability of the project by testing it on various platforms, configurations, and scenarios. Report any issues you encounter, along with detailed information on how to reproduce them.

By following these guidelines, you will help maintain a high level of quality and collaboration in the project, ensuring its continued success and growth.
## License

This project is free to use, but you must give proper reference to the original authors when utilizing any part of it in your work. By using this project, you acknowledge and agree to this requirement. Please make sure to cite our work appropriately to give credit to our team.

```
Eflatun (2023). Object Detection and Tracking on Fixed Wing Airplanes. Version 0.0.1. [Source Code]. Available at: https://github.com/sezer-muhammed/eflatun_src
```
## Hardware Used
In this project, we utilized the following hardware components to achieve object detection and tracking on fixed wing airplanes:

- **Airframe**: Our custom-built airframe has a wingspan of X meters and a length of Y meters. The airframe is made of lightweight, durable materials such as carbon fiber and foam, ensuring optimal flight performance and endurance.

- **Flight Controller**: We used the Flight Controller Model ABC, which features advanced flight control algorithms, GPS waypoint navigation, and compatibility with a wide range of sensors and peripherals.

- **Camera**: Our chosen camera is the Camera Model XYZ with a resolution of 4K at 30 FPS. This high-quality camera ensures clear images and videos, facilitating accurate object detection and tracking. The camera is equipped with a wide dynamic range and low-latency video transmission for real-time analysis.

- **Onboard Computer**: We integrated the Onboard Computer Model 123 with our system, featuring a powerful processor and ample RAM for running machine learning models and processing data in real-time. The onboard computer is responsible for handling object detection and tracking tasks while communicating with the flight controller for efficient decision-making.

- **Sensors**: In addition to the camera, we have incorporated the following sensors to enhance the system's capabilities:
  - GPS: For precise location tracking and waypoint navigation.
  - IMU (Inertial Measurement Unit): For measuring linear and angular motion, enabling accurate attitude estimation and stabilization.
  - LIDAR: For real-time distance measurements, allowing the airplane to maintain a safe distance from obstacles and terrain.

Make sure you have compatible hardware components when using this project in your own fixed wing airplane setup. Detailed specifications and connection diagrams can be found in the documentation.

## Sources
