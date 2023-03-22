from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bag_file_arg = DeclareLaunchArgument(
        'bag_file',
        default_value='/home/sezer/uav_ws/bag_files/rosbag2_2023_03_22-13_22_24',
        description='Path to the bag file.'
    )

    bag_file = LaunchConfiguration('bag_file')

    play_rosbag_node = Node(
        package='ros2bag',  # Update this to 'rosbag2'
        executable='play',
        name='rosbag2_play',
        output='screen',
        arguments=['--ros-args', '--params-file', bag_file]
    )


    best_object_selector_node = Node(
        package='eflatun',
        executable='best_object_selector',
        name='best_object_selector',
        output='screen',
    )

    object_tracker_node = Node(
        package='eflatun',
        executable='object_tracker',
        name='object_tracker',
        output='screen',
    )

    return LaunchDescription([
        bag_file_arg,
        play_rosbag_node,
        best_object_selector_node,
        object_tracker_node,
    ])
