from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    os.system("sudo chmod a+rw /dev/ttyACM0")
    os.system("sudo usermod -a -G dialout $USER")

    return LaunchDescription([
        Node(
            package='mavros',
            executable='mavros_node',
            output='screen',
            parameters=[
                {"fcu_url": "/dev/ttyACM1:115200"},
            ]
        )
    ])
