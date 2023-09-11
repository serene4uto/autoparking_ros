import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2

class SimDataVehicle(Node):
    def __init__(self):
        super().__init__("sim_data_vehicle")

        self.vstt_pub = self.create_publisher(String, "aps/vehicle_status", 10)
        self.vcmd_sub = self.create_subscription(String, "aps/vehicle_cmd", self.on_vehicle_cmd_receive, 10)
        self.camevent_sub = self.create_subscription(String, "aps_simdata/cam_event", self.on_simcam_event_receive, 10)
        self.vsst_pub_timer = self.create_timer(5, self.on_vsst_pub_timer)

        self.license_plate = "AB123"
        self.current_slot = "NoData"
        self.current_vstt = "ready-to-park"

    
    def on_vehicle_cmd_receive(self, vcmd_msg):
        vcmd_msg_split = vcmd_msg.data.split(',')

        if vcmd_msg_split[1] != self.license_plate:
            return
        
        if vcmd_msg_split[0] == "park":
            self.current_vstt = "on-parking"
            self.current_slot = vcmd_msg_split[2]
            
        if vcmd_msg_split[0] == "leave":
            self.current_vstt = "on-leaving"
        
        


    def on_simcam_event_receive(self, simcam_msg):
        simcam_msg_data = simcam_msg.data
        if simcam_msg_data == "on-parking-done":
            self.current_vstt = f"parked"
        if simcam_msg_data == "on-leaving-done":
            # self.current_vstt = "not-parked"
            self.current_slot = "NoData"
            self.current_vstt = "ready-to-park"


    def on_vsst_pub_timer(self):
        vstt_msg = String()
        vstt_msg.data = f"{self.license_plate},{self.current_vstt},{self.current_slot}"
        self.vstt_pub.publish(vstt_msg)


def main(args=None):
    rclpy.init(args=args)
    sdv_node = SimDataVehicle()
    rclpy.spin(sdv_node)
    sdv_node.destroy_node()
    rclpy.shutdown()
    