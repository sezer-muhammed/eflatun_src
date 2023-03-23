import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

import json


import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel, QGraphicsView, QGraphicsScene, QProgressBar
from PyQt5.QtCore import Qt

class UAVGUI(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("UAV Control")

        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        layout = QVBoxLayout(central_widget)

        # Mode buttons and Arm button
        mode_layout = QHBoxLayout()
        layout.addLayout(mode_layout)

        track_button = QPushButton("Track")
        mode_layout.addWidget(track_button)

        rc_button = QPushButton("RC")
        mode_layout.addWidget(rc_button)

        search_button = QPushButton("Search")
        mode_layout.addWidget(search_button)

        arm_button = QPushButton("Arm")
        mode_layout.addWidget(arm_button)

        takeoff_button = QPushButton("Auto Takeoff")
        mode_layout.addWidget(takeoff_button)

        landing_button = QPushButton("Auto Landing")
        mode_layout.addWidget(landing_button)

        # Visual area
        visual_layout = QHBoxLayout()
        layout.addLayout(visual_layout)

        self.view = QGraphicsView()
        self.scene = QGraphicsScene()
        self.view.setScene(self.scene)
        visual_layout.addWidget(self.view)

        self.map_view = QLabel("Map view placeholder")
        visual_layout.addWidget(self.map_view)

        # Telemetry dashboard
        telemetry_layout = QHBoxLayout()
        layout.addLayout(telemetry_layout)

        self.connection_label = QLabel("Connection: ")
        telemetry_layout.addWidget(self.connection_label)

        self.gps_satellites_label = QLabel("GPS: ")
        telemetry_layout.addWidget(self.gps_satellites_label)

        self.wind_speed_label = QLabel("Wind: ")
        telemetry_layout.addWidget(self.wind_speed_label)

        self.distance_label = QLabel("Distance: ")
        telemetry_layout.addWidget(self.distance_label)

        self.flight_time_label = QLabel("Flight Time: ")
        telemetry_layout.addWidget(self.flight_time_label)

        self.flight_mode_label = QLabel("Flight Mode: ")
        telemetry_layout.addWidget(self.flight_mode_label)

        # Battery, Angles, and Status
        battery_angles_status_layout = QHBoxLayout()
        layout.addLayout(battery_angles_status_layout)

        # Battery group
        battery_layout = QVBoxLayout()
        battery_angles_status_layout.addLayout(battery_layout)
        battery_layout.addWidget(QLabel("Battery:"))
        self.battery_percentage_label = QLabel("Percentage: ")
        battery_layout.addWidget(self.battery_percentage_label)
        self.battery_voltage_label = QLabel("Voltage: ")
        battery_layout.addWidget(self.battery_voltage_label)
        self.battery_current_label = QLabel("Current: ")
        battery_layout.addWidget(self.battery_current_label)

        # Angles group
        angles_layout = QVBoxLayout()
        battery_angles_status_layout.addLayout(angles_layout)
        angles_layout.addWidget(QLabel("Angles:"))
        self.yaw_label = QLabel("Yaw: ")
        angles_layout.addWidget(self.yaw_label)
        self.pitch_label = QLabel("Pitch: ")
        angles_layout.addWidget(self.pitch_label)
        self.roll_label = QLabel("Roll: ")
        angles_layout.addWidget(self.roll_label)

        # Status group
        status_layout = QVBoxLayout()
        battery_angles_status_layout.addLayout(status_layout)
        status_layout.addWidget(QLabel("Status:"))
        self.gps_fix_label = QLabel("GPS Fix: ")
        status_layout.addWidget(self.gps_fix_label)
        self.magnetometer_label = QLabel("Magnetometer: ")
        status_layout.addWidget(self.magnetometer_label)
        self.barometer_label = QLabel("Barometer: ")
        status_layout.addWidget(self.barometer_label)

        # GPS Coordinates
        gps_coordinates_layout = QHBoxLayout()
        layout.addLayout(gps_coordinates_layout)
        self.latitude_label = QLabel("Latitude: ")
        gps_coordinates_layout.addWidget(self.latitude_label)

        self.longitude_label = QLabel("Longitude: ")
        gps_coordinates_layout.addWidget(self.longitude_label)

        # Bars
        bars_layout = QHBoxLayout()
        layout.addLayout(bars_layout)

        self.battery_bar = QProgressBar()
        self.battery_bar.setOrientation(Qt.Vertical)
        bars_layout.addWidget(self.battery_bar)

        self.yaw_bar = QProgressBar()
        self.yaw_bar.setOrientation(Qt.Vertical)
        bars_layout.addWidget(self.yaw_bar)

        self.pitch_bar = QProgressBar()
        self.pitch_bar.setOrientation(Qt.Vertical)
        bars_layout.addWidget(self.pitch_bar)

        self.roll_bar = QProgressBar()
        self.roll_bar.setOrientation(Qt.Vertical)
        bars_layout.addWidget(self.roll_bar)

        self.wind_speed_bar = QProgressBar()
        self.wind_speed_bar.setOrientation(Qt.Vertical)
        bars_layout.addWidget(self.wind_speed_bar)

        self.distance_bar = QProgressBar()
        self.distance_bar.setOrientation(Qt.Vertical)
        bars_layout.addWidget(self.distance_bar)

        self.flight_time_bar = QProgressBar()
        self.flight_time_bar.setOrientation(Qt.Vertical)
        bars_layout.addWidget(self.flight_time_bar)

# Main function to run the application
def main():
    app = QApplication(sys.argv)
    main_window = UAVGUI()
    main_window.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()