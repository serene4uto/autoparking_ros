import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from threading import Thread
import requests

# server_address = "http://127.0.0.1:5000"
# api_post_vehicle_stt = "apiv1/vehicle/status"
# api_get_vehicle_cmd = "apiv1/vehicle/command"
# api_post_parking_slot_stt = "apiv1/parking_slot/status"

class Bridge2Server(Node):
    def __init__(self):
        super().__init__("bridge_2_server")

        # Set default parameters
        self.declare_parameter('server_url', 'http://127.0.0.1:5000')
        self.declare_parameter('api_endpoint_vehicle_stt', 'apiv1/vehicle/status')
        self.declare_parameter('api_endpoint_vehicle_cmd', 'apiv1/vehicle/command')
        self.declare_parameter('api_endpoint_parkinglot_stt', 'apiv1/parking_slot/status')

        # Get parameters
        self.server_address = self.get_parameter('server_url').value
        self.api_post_vehicle_stt = self.get_parameter('api_endpoint_vehicle_stt').value
        self.api_get_vehicle_cmd = self.get_parameter('api_endpoint_vehicle_cmd').value
        self.api_post_parking_slot_stt = self.get_parameter('api_endpoint_parkinglot_stt').value
        

        self.vcmd_pub = self.create_publisher(String, "aps/vehicle_cmd", 10)
        self.vstt_sub = self.create_subscription(String, "aps/vehicle_status", self.on_vehicle_stt_receive, 10)
        self.psstt_sub = self.create_subscription(String, "aps/parking_slot_status", self.on_parking_slot_stt_receive, 10)

        self.long_polling_vehicle_cmd_thread = Thread( 
                        target=self.longpoll_vehicle_cmd_thread,
                        daemon=True)
        
        self.long_polling_vehicle_cmd_thread.start()

    def longpoll_vehicle_cmd_thread(self):
        while rclpy.ok():
            try:
                response = requests.get(f"{self.server_address}/{self.api_get_vehicle_cmd}")
                if response.status_code == 200:
                    self.get_logger().info('GET request for vehicle command successful')
                    msgs = response.json()

                    if "message" not in msgs:
                        # public
                        vcmd_msg = String()
                        vcmd_msg.data = f"{msgs['command']},{msgs['license_plate']},{msgs['parking_slot']}"
                        self.vcmd_pub.publish(vcmd_msg)
            except requests.exceptions.RequestException as e:
                self.get_logger().error(f'An error occurred: {e}')

    def on_vehicle_stt_receive(self, vstt_msg):
        vstt_msg_split = vstt_msg.data.split(',')
        vehicle_stt_data = {
            "license_plate" : vstt_msg_split[0],
            "status" : vstt_msg_split[1],
        }

        try:
            response = requests.post(url=f"{self.server_address}/{self.api_post_vehicle_stt}", json=vehicle_stt_data, timeout=None)

            if response.status_code == 200:
                self.get_logger().info('POST request for vehicle status successful')
                self.get_logger().info(f'Response content: {response.text}')
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f'An error occurred: {e}')
    

    def on_parking_slot_stt_receive(self, psstt_msg):
        psstt_msg_split = psstt_msg.data.split(',')

        parking_slot_stt_data = {
            "parking_slot" : psstt_msg_split[0],
            "status" : psstt_msg_split[1],
        }

        try:
            response = requests.post(url=f"{self.server_address}/{self.api_post_parking_slot_stt}", json=parking_slot_stt_data, timeout=None)

            if response.status_code == 200:
                self.get_logger().info('POST request for parking slot status successful')
                self.get_logger().info(f'Response content: {response.text}')
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f'An error occurred: {e}')
            


def main(args=None):
    rclpy.init(args=args)
    b2s_node = Bridge2Server()
    rclpy.spin(b2s_node)
    b2s_node.destroy_node()
    rclpy.shutdown()
