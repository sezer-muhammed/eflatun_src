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
            executable='vehicle_status_gui',
            name='vehicle_status_gui',
            output='screen',
            parameters=[os.path.join(get_package_share_directory('eflatun'),'config','vehicle_status_gui.yaml')]
        ),
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', '/home/sezer/uav_ws/bag_files/rosbag2_2023_03_22-19_33_53'],
            output='screen'
        ),
    ])
