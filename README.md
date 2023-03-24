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
13. [Contact Information](#contact-information)

## Introduction

The introduction section should provide a brief overview of the main features, capabilities, and use cases of the package. Explain the purpose of the package and what it aims to achieve.

## Dependencies

In the Dependencies section, list and explain all external dependencies required for the package to work, including specific versions if necessary. Add instructions for installing these dependencies.

## Installation

The Installation section should provide step-by-step installation instructions for the package. Include information on installing the package from source or using a package manager, if applicable.

## Usage

The Usage section should include detailed examples demonstrating how to use the package. Provide code snippets, terminal commands, or configuration files to help users understand the package's functionality better.

## Node Descriptions

### object_detector

In this section, provide a detailed description of the `object_detector` node, explaining its purpose and functionality.

#### Published Topics

List and describe the topics published by the `object_detector` node.

#### Subscribed Topics

List and describe the topics to which the `object_detector` node subscribes.

### object_tracker

In this section, provide a detailed description of the `object_tracker` node, explaining its purpose and functionality.

#### Published Topics

List and describe the topics published by the `object_tracker` node.

#### Subscribed Topics

List and describe the topics to which the `object_tracker` node subscribes.

### best_object_selector

In this section, provide a detailed description of the `best_object_selector` node, explaining its purpose and functionality.

#### Published Topics

List and describe the topics published by the `best_object_selector` node.

#### Subscribed Topics

List and describe the topics to which the `best_object_selector` node subscribes.

### vehicle_status

In this section, provide a detailed description of the `vehicle_status` node, explaining its purpose and functionality.

#### Published Topics

List and describe the topics published by the `vehicle_status` node.

#### Subscribed Topics

List and describe the topics to which the `vehicle_status` node subscribes.

## Parameters

In the Parameters section, use tables to organize the parameters for better readability. For each parameter, include its name, a brief description, and its default value.

## Messages

In the Messages section, use tables to organize the custom messages. For each message, include its name, fields, and a brief description of its purpose.

## Code Examples

The Code Examples section should provide examples demonstrating how to use the package in various scenarios. Include code snippets or configuration files to give users a clear understanding of how to implement the package in their projects.

## Troubleshooting

In the Troubleshooting section, include common issues users might face and their solutions. This section will help users quickly identify and resolve any problems they encounter while using the package.

## Contributing

Add guidelines for contributing to the project, such as submitting issues, creating pull requests, or participating in the development process.

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

## Contact Information

If you have any questions or need support, feel free to reach out to our team members:

- **Şevval Belkıs Dikkaya**: Head of the team, working on software, electronics, and mechanics. [LinkedIn](https://www.linkedin.com/in/sbdikkaya/)
- **Muhammed Sezer**: Responsible for electronics and software, and the maintainer of this package. [LinkedIn](https://www.linkedin.com/in/muhammed-sezer-160428208/)
- **Metehan İçöz**: Responsible for mechanics. [LinkedIn](https://www.linkedin.com/in/metehan-içöz-735bab206/)
- **Zeynep Keleş**: Working on software, electronics, and mechanics. [LinkedIn](https://www.linkedin.com/in/zeynep-bilge-keleş-7ba917255/)

Feel free to connect with us on LinkedIn and mention any specific questions or issues related to the project. We'll be happy to help!
