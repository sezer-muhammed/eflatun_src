from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='eflatun',
            executable='fake_object_detector',
            name='fake_object_detector',
            output='screen',
            parameters=[os.path.join(get_package_share_directory('eflatun'),'config','object_detector.yaml')]
        ),
    ])