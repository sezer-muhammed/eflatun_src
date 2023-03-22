from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():


    play_rosbag_node = ExecuteProcess(
            cmd=['ros2', 'bag', 'play', '-a'],
            output='screen'
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
        #play_rosbag_node,
        best_object_selector_node,
        object_tracker_node,
    ])
