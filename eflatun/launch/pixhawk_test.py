from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mavros',
            executable='mavros_node',
            name='mavros',
            output='screen',
            parameters=[
                {'fcu_url': 'serial:///dev/ttyACM0:115200'},
                {'system_id': 1},
                {'component_id': 1},
                {'target_system': 1},
                {'target_component': 1}
            ]
        )
    ])
