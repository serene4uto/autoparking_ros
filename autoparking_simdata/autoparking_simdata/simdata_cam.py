import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image

import os

import cv2
from cv_bridge import CvBridge

FAKE_CAM_SLOT_DATA = ["A1", "A2", "A3", "A4"]
FRAME_RATE = 50

class SimDataCam(Node):
    def __init__(self):
        super().__init__("sim_data_cam_node")

        # # Set default parameters
        self.declare_parameter('data_path', '')

        # Get parameters
        self.sim_data_path = self.get_parameter('data_path').get_parameter_value().string_value

        # self.sim_data_path = "/workspaces/Project_APS_new"


        self.FAKE_CAM_DATA_PATH = {
            "not-parked"  : os.path.join(self.sim_data_path, "fake_cam_data/AB123/not_parked.mp4"),

            "on-parking" : [os.path.join(self.sim_data_path, "fake_cam_data/AB123/A1_onparking.mp4"),
                            os.path.join(self.sim_data_path, "fake_cam_data/AB123/A2_onparking.mp4"),
                            os.path.join(self.sim_data_path, "fake_cam_data/AB123/A3_onparking.mp4"),
                            os.path.join(self.sim_data_path, "fake_cam_data/AB123/A4_onparking.mp4"),],

            "parked"     : [os.path.join(self.sim_data_path, "fake_cam_data/AB123/A1_parked.mp4"),
                            os.path.join(self.sim_data_path, "fake_cam_data/AB123/A2_parked.mp4"),
                            os.path.join(self.sim_data_path, "fake_cam_data/AB123/A3_parked.mp4"),
                            os.path.join(self.sim_data_path, "fake_cam_data/AB123/A4_parked.mp4"),],

            "on-leaving" : [os.path.join(self.sim_data_path, "fake_cam_data/AB123/A1_onleaving.mp4"),
                            os.path.join(self.sim_data_path, "fake_cam_data/AB123/A2_onleaving.mp4"),
                            os.path.join(self.sim_data_path, "fake_cam_data/AB123/A3_onleaving.mp4"),
                            os.path.join(self.sim_data_path, "fake_cam_data/AB123/A4_onleaving.mp4"),]
        }        

        self.vstt_sub = self.create_subscription(String, "aps/vehicle_status", self.on_vehicle_stt_receive, 10)
        self.camevent_pub = self.create_publisher(String, "aps_simdata/cam_event", 10)
        self.fakecam_image_pub = self.create_publisher(Image, "aps/parkinglot_cam/fake_data", 10)

        self.sdc_timer = self.create_timer(1/FRAME_RATE, self.on_timer)

        self.bridge = CvBridge()
        self.current_vcap = cv2.VideoCapture(self.FAKE_CAM_DATA_PATH["not-parked"])

        self.license_plate = "AB123"

        self.current_vstt = "not-parked"
        self.vstt_change = False
        self.current_slot = None


        
    def on_vehicle_stt_receive(self, vstt_msg):
        vstt_msg_split = vstt_msg.data.split(',')

        if vstt_msg_split[0] != self.license_plate:
            return
        
        # if len(vstt_msg_split) == 3:
        #     self.current_slot = vstt_msg_split[2]
        # else:
        #     self.current_slot = None

        if vstt_msg_split[2] == "NoData":
            self.current_slot = None
        else:
            self.current_slot = vstt_msg_split[2]
        
        if self.current_vstt != vstt_msg_split[1]:
            self.vstt_change = True
            self.current_vstt = vstt_msg_split[1]



    def on_timer(self):
        try:
            if self.vstt_change == True:
                self.current_vcap.release()
                if self.current_vstt == "not-parked" or self.current_vstt == "ready-to-park":
                    self.current_vcap = cv2.VideoCapture(self.FAKE_CAM_DATA_PATH["not-parked"])

                if self.current_vstt == "on-parking":
                    self.current_vcap = cv2.VideoCapture(self.FAKE_CAM_DATA_PATH["on-parking"][FAKE_CAM_SLOT_DATA.index(self.current_slot)])

                if self.current_vstt == "on-leaving":
                    self.current_vcap = cv2.VideoCapture(self.FAKE_CAM_DATA_PATH["on-leaving"][FAKE_CAM_SLOT_DATA.index(self.current_slot)])

                if self.current_vstt == "parked" :
                    self.current_vcap = cv2.VideoCapture(self.FAKE_CAM_DATA_PATH["parked"][FAKE_CAM_SLOT_DATA.index(self.current_slot)])

                self.vstt_change = False


            ret, frame = self.current_vcap.read()   

            if not ret:

                if self.current_vstt == "not-parked" or self.current_vstt == "ready-to-park" or self.current_vstt == "parked" :
                    self.current_vcap.set(cv2.CAP_PROP_POS_FRAMES, 0)

                if self.current_vstt == "on-parking":
                    simcam_event_msg = String()
                    simcam_event_msg.data = "on-parking-done"
                    self.camevent_pub.publish(simcam_event_msg)

                if self.current_vstt == "on-leaving":
                    simcam_event_msg = String()
                    simcam_event_msg.data = "on-leaving-done"
                    self.camevent_pub.publish(simcam_event_msg)

                return

            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.fakecam_image_pub.publish(img_msg)
            # self.get_logger().info("Public Fake Image Data")
        except Exception  as e:
            self.get_logger().error(f"An exception of type {type(e).__name__} occurred: {e}")


def main(args=None):
    rclpy.init(args=args)
    sdc_node = SimDataCam()
    rclpy.spin(sdc_node)
    sdc_node.destroy_node()
    rclpy.shutdown()

        

