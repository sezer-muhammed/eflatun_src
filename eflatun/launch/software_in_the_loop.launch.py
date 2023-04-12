from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='eflatun',
            executable='object_tracker',
            name='object_tracker',
            output='screen',
            parameters=[os.path.join(get_package_share_directory('eflatun'),'config','object_tracker.yaml')]
        ),
        Node(
            package='eflatun',
            executable='best_object_selector',
            name='best_object_selector',
            output='screen',
            parameters=[os.path.join(get_package_share_directory('eflatun'),'config','best_object_selector.yaml')]
        ),
        Node(
            package='eflatun',
            executable='controller',
            name='controller',
            output='screen',
            parameters=[os.path.join(get_package_share_directory('eflatun'),'config','controller.yaml')]
        ),
        Node(
            package='eflatun',
            executable='object_detector',
            name='object_detector',
            output='screen',
            parameters=[os.path.join(get_package_share_directory('eflatun'),'config','object_detector.yaml')]
        ),
#        ExecuteProcess(
#            cmd=['ros2', 'bag', 'play', '/home/sezer/Desktop/uav_ws/eflatun_src/bag_files/rosbag2_2023_03_22-19_33_53'],
#            output='screen'
#        ),
    ])
