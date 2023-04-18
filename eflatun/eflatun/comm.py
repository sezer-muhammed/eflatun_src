import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import requests
import time
import json

from mavros_msgs.msg import State, OverrideRCIn
from sensor_msgs.msg import NavSatFix, Imu, BatteryState
from geometry_msgs.msg import TwistStamped, PoseStamped
from std_msgs.msg import Float64
from eflatun_msgs.msg import TrackedObject, TrackedObjectArray
from rclpy.qos import QoSProfile, qos_profile_sensor_data

import math

class ServerCommunicationNode(Node):
    def __init__(self):
        super().__init__('server_communication_node')
        qos = QoSProfile(depth=2)
        self.subscription_state = self.create_subscription(BatteryState, '/mavros/battery', self.state_callback, qos_profile_sensor_data)
        self.subscription_gps = self.create_subscription(NavSatFix, 'mavros/global_position/global', self.gps_callback, qos_profile_sensor_data)
        self.subscription_velocity = self.create_subscription(Float64, '/mavros/global_position/compass_hdg', self.heading, qos_profile_sensor_data)
        self.subscription_pose = self.create_subscription(Imu, 'mavros/imu/data', self.pose_callback, qos_profile_sensor_data)

        #self.subscription_target = self.create_publisher(TrackedObject, '/tracker/best_object', self.target_callback, qos_profile_sensor_data)
        self.mavros_velocity_sub = self.create_subscription(TwistStamped, '/mavros/local_position/velocity_local', self.velocity_callback, qos_profile_sensor_data)
        self.subscription_override = self.create_subscription(OverrideRCIn, "/mavros/rc/override", self.override_callback, qos_profile_sensor_data)

        self.session = requests.Session()

        self.kilit_cooldown = 0
        self.otonom_cooldown = 0
        self.url = 'http://10.0.0.10:10001'
        self.username = 'eflatunuav'
        self.password = 'a7rd5Dzdfu'

        self.login()

        self.telemetry_data = {
            "takim_numarasi": 5,  # Replace with your actual team number
            "iha_enlem": 0,
            "iha_boylam": 0,
            "iha_irtifa": 0,
            "iha_dikilme": 0,
            "iha_yonelme": 0,
            "iha_yatis": 0,
            "iha_hiz": 0,
            "iha_batarya": 50,  # Replace with your actual battery data
            "iha_otonom": 1,
            "iha_kilitlenme": 1,  # Update with your actual locking data
            "hedef_merkez_X": 300,  # Update with your actual target data
            "hedef_merkez_Y": 230,
            "hedef_genislik": 30,
            "hedef_yukseklik": 43,
            "gps_saati": {
                "saat": 0,
                "dakika": 0,
                "saniye": 0,
                "milisaniye": 0
            }
        }

        self.otonom_kilitlenme = {
            "kilitlenmeBaslangicZamani": {
                "saat": 11,
                "dakika": 40,
                "saniye": 51,
                "milisaniye": 478
            },
            "kilitlenmeBitisZamani": {
                "saat": 11,
                "dakika": 41,
                "saniye": 3,
                "milisaniye": 141
            },
            "otonom_kilitlenme": 1
            }
        
        self.kamikaze_data = {
            "kamikazeBaslangicZamani": {
                "saat": 11,
                "dakika": 44,
                "saniye": 13,
                "milisaniye": 361
            },
            "kamikazeBitisZamani": {
                "saat": 11,
                "dakika": 44,
                "saniye": 27,
                "milisaniye": 874
            },
            "qrMetni": "asparagasaspargasarapatagastersmakas"
            }
        
        self.create_timer(0.8, self.send_telemetry)



    def login(self):
        self.user_data = {
            'kadi': self.username,
            'sifre': self.password
        }
        headers = {'Content-Type': 'application/json'}
        response = self.session.post(f"{self.url}/api/giris", json=self.user_data, headers=headers)
        if response.status_code == 200:
            self.team_number = response.text
            self.get_logger().info(f'Team number: {self.team_number}')
        else:
            self.get_logger().error(f'Error connecting to server: {response.status_code} - {response.text}')
            rclpy.shutdown()

    def override_callback(self, msg):
        self.otonom_cooldown = 2

    def velocity_callback(self, msg):
        self.telemetry_data["iha_hiz"] = round(math.sqrt(msg.twist.linear.x**2+msg.twist.linear.y**2+msg.twist.linear.z**2), 3)

    def listener_callback(self, msg):
        self.get_logger().info(f'Received message: "{msg.data}"')

    def send_telemetry(self):
        self.get_logger().info(f"{self.telemetry_data}")
        self.kilit_cooldown -= 1
        self.otonom_cooldown -= 1
        self.otonom_cooldown = max(self.otonom_cooldown, 0)
        self.kilit_cooldown = max(self.kilit_cooldown, 0)
        if self.kilit_cooldown == 0:
            self.telemetry_data["iha_kilitlenme"] = 0
        if self.otonom_cooldown == 0:
            self.telemetry_data["iha_otonom"] = 0
        self.get_sunucu_saati()
        headers = {'Content-Type': 'application/json'}
        response = self.session.post(f"{self.url}/api/telemetri_gonder", json=self.telemetry_data, headers=headers)

        if response.status_code == 200:
            other_teams_data = json.loads(response.text)
            self.get_logger().info("telemetry data sent")
            return other_teams_data
        else:
            print(f'Error connecting to server: {response.status_code} - {response.text}')
            return None

    def target_callback(self, msg):
        self.telemetry_data["hedef_merkez_X"] = msg.center_x
        self.telemetry_data["hedef_merkez_Y"] = msg.center_y
        self.telemetry_data["hedef_genislik"] = msg.width
        self.telemetry_data["hedef_yukseklik"] = msg.height

        self.telemetry_data["iha_otonom"] = 1
        self.telemetry_data["iha_kilitlenme"] = 1

        self.kilit_cooldown = 2

    def get_sunucu_saati(self):

        url = f"{self.url}/api/sunucusaati"
        response = self.session.get(url)

        if response.status_code == 200:
            server_time = json.loads(response.text)
            server_time["milisaniye"] = max(server_time["milisaniye"] - 60, 0)
            self.telemetry_data["gps_saati"] = server_time
            return server_time
        else:
            print(f'Error connecting to server: {response.status_code} - {response.text}')
            return None

    def send_kilitlenme_bilgisi(self, kilitlenme_data):
        
        headers = {'Content-Type': 'application/json'}
        kilitlenme_bilgisi_url = f"{self.url}/api/kilitlenme_bilgisi"
        
        response = self.session.post(kilitlenme_bilgisi_url, json=kilitlenme_data, headers=headers)

        if response.status_code == 200:
            print("GONDERİLDİ")
            return None
        else:
            print(f'Error connecting to server: {response.status_code} - {response.text}')
            return None

    def send_kamikaze_bilgisi(self, kamikaze_data):
        headers = {'Content-Type': 'application/json'}
        kamikaze_bilgisi_url = f"{self.url}/api/kamikaze_bilgisi"
        
        response = self.session.post(kamikaze_bilgisi_url, json=kamikaze_data, headers=headers)
        
        if response.status_code == 200:
            print("GONDERİLDİ")
            return None
        else:
            print(f'Error connecting to server: {response.status_code} - {response.text}')
            return None

    def get_qr_koordinati(self):
        qr_koordinati_url = f"{self.url}/api/qr_koordinati"
        
        response = self.session.get(qr_koordinati_url)

        if response.status_code == 200:
            qr_koordinati_data = json.loads(response.text)
            return qr_koordinati_data
        else:
            print(f'Error connecting to server: {response.status_code} - {response.text}')
            return None
        
    def state_callback(self, msg):
        self.telemetry_data["iha_batarya"] = int(msg.percentage * 100)
        

    def gps_callback(self, msg):
        # Update telemetry data with current GPS position and time
        latitude = msg.latitude
        longitude = msg.longitude
        altitude = msg.altitude - 136.55
        # Update telemetry data accordingly
        self.telemetry_data["iha_boylam"] = longitude
        self.telemetry_data["iha_enlem"] = latitude
        self.telemetry_data["iha_irtifa"] = altitude

        #self.get_logger().warn("GPS Callback alındı")

    def heading(self, msg):

        self.telemetry_data["iha_yonelme"] = int(msg.data)

        #self.get_logger().warn("velocity Callback alındı")



    def pose_callback(self, msg):
        # Update telemetry data with current orientation
        roll, pitch, yaw = self.quaternion_to_euler(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        # Update telemetry data accordingly
        self.telemetry_data["iha_yatis"] = int(roll * 180 / 3.1415926)
        self.telemetry_data["iha_dikilme"] = int(pitch * 180 / 3.1415926) *-1

    def quaternion_to_euler(self, x, y, z, w):
        # Convert quaternion to Euler angles
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)

        return roll, pitch, yaw

def main(args=None):
    rclpy.init(args=args)
    login_node = ServerCommunicationNode()
    rclpy.spin(login_node)

    login_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()