from rclpy.qos import QoSProfile, qos_profile_sensor_data
import rclpy
from rclpy.node import Node
import threading

from diagnostic_msgs.msg import DiagnosticArray
from sensor_msgs.msg import BatteryState, Imu, FluidPressure, MagneticField, NavSatFix
from mavros_msgs.msg import RCIn, RCOut, NavControllerOutput
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import UInt32

import sys
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QTableWidget, QTableWidgetItem

class MavrosSubscriber(Node):

    def __init__(self):
        super().__init__('multi_topic_subscriber')

        # Subscribing to the provided topics
        self.create_subscription(DiagnosticArray, '/diagnostics', self.update_diagnostics, qos_profile_sensor_data)
        self.create_subscription(BatteryState, '/mavros/battery', self.update_battery, qos_profile_sensor_data)
        self.create_subscription(Imu, '/mavros/mavros/data', self.update_data, qos_profile_sensor_data)
        self.create_subscription(Imu, '/mavros/mavros/data_raw', self.update_data_raw, qos_profile_sensor_data)
        self.create_subscription(FluidPressure, '/mavros/mavros/diff_pressure', self.update_diff_pressure, qos_profile_sensor_data)
        self.create_subscription(RCIn, '/mavros/mavros/in', self.update_in, qos_profile_sensor_data)
        self.create_subscription(MagneticField, '/mavros/mavros/mag', self.update_mag, qos_profile_sensor_data)
        self.create_subscription(RCOut, '/mavros/mavros/out', self.update_out, qos_profile_sensor_data)
        self.create_subscription(NavControllerOutput, '/mavros/mavros/output', self.update_output, qos_profile_sensor_data)
        self.create_subscription(NavSatFix, '/mavros/mavros/raw/fix', self.update_raw_fix, qos_profile_sensor_data)
        self.create_subscription(TwistStamped, '/mavros/mavros/raw/gps_vel', self.update_raw_gps_vel, qos_profile_sensor_data)
        self.create_subscription(UInt32, '/mavros/mavros/raw/satellites', self.update_raw_satellites, qos_profile_sensor_data)

        x = threading.Thread(target=self.threading_gui, args=())
        x.start()

    def threading_gui(self):
        self.app = QApplication(sys.argv)
        self.mavros_gui = MavrosGUI()
        self.mavros_gui.run()
        sys.exit(self.app.exec_())

    def update_diagnostics(self, msg):
        self.mavros_gui.update_table(0, msg)

    def update_battery(self, msg):
        self.mavros_gui.update_table(1, msg)

    def update_data(self, msg):
        self.mavros_gui.update_table(2, msg)

    def update_data_raw(self, msg):
        self.mavros_gui.update_table(3, msg)

    def update_diff_pressure(self, msg):
        self.mavros_gui.update_table(4, msg)

    def update_in(self, msg):
        self.mavros_gui.update_table(5, msg)

    def update_mag(self, msg):
        self.mavros_gui.update_table(6, msg)

    def update_out(self, msg):
        self.mavros_gui.update_table(7, msg)

    def update_output(self, msg):
        self.mavros_gui.update_table(8, msg)

    def update_raw_fix(self, msg):
        self.mavros_gui.update_table(9, msg)

    def update_raw_gps_vel(self, msg):
        self.mavros_gui.update_table(10, msg)

    def update_raw_satellites(self, msg):
        self.mavros_gui.update_table(11, msg)



class MavrosGUI(QWidget):

    def __init__(self):
        super().__init__()

        # Set up the user interface
        self.init_ui()

    def init_ui(self):
        self.setWindowTitle('Mavros Data Visualization')
        self.setGeometry(100, 100, 800, 400)

        layout = QVBoxLayout()

        self.table = QTableWidget(12, 2)
        self.table.setHorizontalHeaderLabels(['Topic', 'Value'])
        self.table.setVerticalHeaderLabels([
            '/diagnostics',
            '/mavros/battery',
            '/mavros/mavros/data',
            '/mavros/mavros/data_raw',
            '/mavros/mavros/diff_pressure',
            '/mavros/mavros/in',
            '/mavros/mavros/mag',
            '/mavros/mavros/out',
            '/mavros/mavros/output',
            '/mavros/mavros/raw/fix',
            '/mavros/mavros/raw/gps_vel',
            '/mavros/mavros/raw/satellites',
        ])
        self.table.horizontalHeader().setStretchLastSection(True)

        layout.addWidget(self.table)
        self.setLayout(layout)

    def update_table(self, row, msg):
        if isinstance(msg, DiagnosticArray):
            value = f"{msg.header.stamp.sec}.{msg.header.stamp.nanosec}: {msg.status[0].message}"
        elif isinstance(msg, BatteryState):
            value = f"{msg.voltage:.2f} V, {msg.percentage:.2f}%"
        elif isinstance(msg, Imu):
            value = f"linear_acceleration={msg.linear_acceleration}, angular_velocity={msg.angular_velocity}, orientation={msg.orientation}"
        elif isinstance(msg, FluidPressure):
            value = f"{msg.fluid_pressure:.2f} Pa"
        elif isinstance(msg, RCIn):
            value = f"{msg.channels}"
        elif isinstance(msg, MagneticField):
            value = f"magnetic_field={msg.magnetic_field}, magnetic_field_covariance={msg.magnetic_field_covariance}"
        elif isinstance(msg, RCOut):
            value = f"{msg.channels}"
        elif isinstance(msg, NavControllerOutput):
            value = f"nav_pitch={msg.nav_pitch}, nav_roll={msg.nav_roll}, nav_bearing={msg.nav_bearing}, target_bearing={msg.target_bearing}, alt_error={msg.alt_error}, aspd_error={msg.aspd_error}, xtrack_error={msg.xtrack_error}"
        elif isinstance(msg, NavSatFix):
            value = f"latitude={msg.latitude:.6f}, longitude={msg.longitude:.6f}, altitude={msg.altitude:.2f} m"
        elif isinstance(msg, TwistStamped):
            value = f"linear={msg.twist.linear}, angular={msg.twist.angular}"
        elif isinstance(msg, UInt32):
            value = f"{msg.data}"
        else:
            value = str(msg)
        self.table.setItem(row, 1, QTableWidgetItem(str(value)))

    def run(self):
        self.show()
        while rclpy.ok():
            QApplication.processEvents()



def main(args=None) -> None:
    rclpy.init(args=args)

    mavros_subscriber = MavrosSubscriber()

    rclpy.spin(mavros_subscriber)

    mavros_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
