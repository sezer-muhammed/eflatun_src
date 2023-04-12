import rclpy
from rclpy.node import Node

from mavros_msgs.msg import OverrideRCIn
from eflatun_msgs.msg import TrackedObject
from rclpy.qos import qos_profile_sensor_data

from rclpy.parameter import Parameter
from rclpy.logging import LoggingSeverity
from rcl_interfaces.msg import SetParametersResult

import json

class ControlPixhawk(Node):
    def __init__(self):
        super().__init__('send_attitude_setpoint')

        self.declare_parameters(namespace='',
                                parameters=[
                                    ('log_level', Parameter.Type.STRING),
                                    ('frame_center.x', Parameter.Type.INTEGER),
                                    ('frame_center.y', Parameter.Type.INTEGER),
                                    ('PID_Aileron.P', Parameter.Type.DOUBLE),
                                    ('PID_Aileron.I', Parameter.Type.DOUBLE),
                                    ('PID_Aileron.D', Parameter.Type.DOUBLE),
                                    ('PID_Aileron.FF', Parameter.Type.DOUBLE),
                                    ('PID_Elevator.P', Parameter.Type.DOUBLE),
                                    ('PID_Elevator.I', Parameter.Type.DOUBLE),
                                    ('PID_Elevator.D', Parameter.Type.DOUBLE),
                                    ('PID_Elevator.FF', Parameter.Type.DOUBLE),
                                    ('PID_Rudder.P', Parameter.Type.DOUBLE),
                                    ('PID_Rudder.I', Parameter.Type.DOUBLE),
                                    ('PID_Rudder.D', Parameter.Type.DOUBLE),
                                    ('PID_Rudder.FF', Parameter.Type.DOUBLE),
                                    ('PID_Thrust.P', Parameter.Type.DOUBLE),
                                    ('PID_Thrust.I', Parameter.Type.DOUBLE),
                                    ('PID_Thrust.D', Parameter.Type.DOUBLE),
                                    ('PID_Thrust.FF', Parameter.Type.DOUBLE)
                                ])

        self.params = {
            'log_level': self.get_parameter('log_level').value,
            'frame_center': {
                'x': self.get_parameter('frame_center.x').value,
                'y': self.get_parameter('frame_center.y').value
            },
            'PID_Aileron': {
                'P': self.get_parameter('PID_Aileron.P').value,
                'I': self.get_parameter('PID_Aileron.I').value,
                'D': self.get_parameter('PID_Aileron.D').value,
                'FF': self.get_parameter('PID_Aileron.FF').value
            },
            'PID_Elevator': {
                'P': self.get_parameter('PID_Elevator.P').value,
                'I': self.get_parameter('PID_Elevator.I').value,
                'D': self.get_parameter('PID_Elevator.D').value,
                'FF': self.get_parameter('PID_Elevator.FF').value
            },
            'PID_Rudder': {
                'P': self.get_parameter('PID_Rudder.P').value,
                'I': self.get_parameter('PID_Rudder.I').value,
                'D': self.get_parameter('PID_Rudder.D').value,
                'FF': self.get_parameter('PID_Rudder.FF').value
            },
            'PID_Thrust': {
                'P': self.get_parameter('PID_Thrust.P').value,
                'I': self.get_parameter('PID_Thrust.I').value,
                'D': self.get_parameter('PID_Thrust.D').value,
                'FF': self.get_parameter('PID_Thrust.FF').value
            }
        }

        log_level_mapping = {
            'debug': LoggingSeverity.DEBUG,
            'info': LoggingSeverity.INFO,
            'warn': LoggingSeverity.WARN,
            'error': LoggingSeverity.ERROR,
            'fatal': LoggingSeverity.FATAL,
        }
        log_level = log_level_mapping.get(self.params["log_level"], LoggingSeverity.INFO)
        self.get_logger().set_level(log_level)

        self.get_logger().info(json.dumps(self.params, sort_keys=True, indent=4))

        self.publisher = self.create_publisher(OverrideRCIn, '/mavros/rc/override', 2)
        self.best_object_callback = self.create_subscription(TrackedObject, "tracker/best_object", self.best_object_callback, qos_profile_sensor_data)

        self.area = 0
        self.width = 0
        self.heights = 0
        self.x = 0
        self.y = 0
        self.id = 0

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def best_object_callback(self, msg: TrackedObject):
        self.area = msg.width * msg.height
        self.width = msg.width
        self.heights = msg.height
        self.x = msg.center_x
        self.y = msg.center_y
        self.id = msg.unique_id

    def timer_callback(self):
        msg = OverrideRCIn()

        error_x = self.x - self.params["frame_center"]["x"]
        error_y = self.y - self.params["frame_center"]["y"]

        msg.channels[0] = int(error_x) + 1000
        msg.channels[1] = int(error_y) + 1000

        self.publisher.publish(msg)


    def on_parameter_change(self, params):
        # Update the parameter dictionary with the new values
        for param in params:
            try:
                param_name = param.name
                param_value = param.value

                param_name = param_name.split(".")
                params_temp = self.params
                for key in param_name[:-1]:
                    params_temp = params_temp[key]
                params_temp[param_name[-1]] = param_value
                self.get_logger().info('Parameter {} updated to {}'.format(param_name, param_value))
            except:
                self.get_logger().warning('Parameter {} cannot updated to {}'.format(param_name, param_value))
                return SetParametersResult(successful=False)
            
        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)
    control_pixhawk = ControlPixhawk()
    rclpy.spin(control_pixhawk)

    control_pixhawk.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
