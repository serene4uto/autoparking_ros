import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image

import datetime

import yaml
import cv2
from ultralytics import YOLO
import numpy as np
from shapely.geometry import Polygon
from cv_bridge import CvBridge

CONFIDENCE_THRESHOLD = 0.5
GREEN = (0, 255, 0)
RED = (0, 0, 255)
BLUE = (255, 0, 0)
ORANGE = (0, 165, 255)
YELLOW = (0, 255, 255)

def rect_to_poly(rect):
    x1, y1, x2, y2 = rect

    x3, y3 = x1, y1
    x4, y4 = x2, y1
    x5, y5 = x2, y2
    x6, y6 = x1, y2

    return [[x3, y3], [x4, y4], [x5, y5], [x6, y6]]


def find_lowest_left_point(points):
    # The image pixel is reversed so the lowest point in real image is the pixel in highest position
    lowest_left_point = None
    lowest_y = float('-inf')  # Initialize with negative infinity to ensure any point will be higher

    for x, y in points:
        if y > lowest_y or (y == lowest_y and x < lowest_left_point[0]):
            lowest_y = y
            lowest_left_point = (x, y)

    return lowest_left_point

def check_occupation_for_slot(slot_mask, detection_boxes):
    max_iou = 0
    occupied_object_id = 'inf'

    # loop over the detections
    for detection_box in detection_boxes:

        maskpoints = slot_mask['points']
        polypts = np.array(maskpoints, np.int32)

        poly_mask = Polygon(polypts)
        poly_det = Polygon(rect_to_poly([int(detection_box[0]), 
                                         int(detection_box[1]), 
                                         int(detection_box[2]), 
                                         int(detection_box[3])]))
        intersect = poly_mask.intersection(poly_det).area
        union = poly_mask.union(poly_det).area
        iou = intersect / union

        if iou > max_iou:
            max_iou = iou
            occupied_object_id = detection_box[6]

    return max_iou, occupied_object_id

class PSlotStatusDetector(Node):
    def __init__(self):
        super().__init__("pslot_status_detector")
        self.pscam_image_sub = self.create_subscription(Image, "aps/parkinglot_cam/fake_data", self.on_parkinglot_cam_image_receive,10)
        self.pslot_stt_pub = self.create_publisher(String, "aps/parking_slot_status", 10)

        self.slotmask_path = "/workspaces/Project_APS_new/fake_cam_data/slot_mask/KNU_fakedata_demo_poly_v1.yaml"
        

        with open(self.slotmask_path, 'r') as file:
            self.slot_masks = yaml.safe_load(file)['slot_mask']

        self.cv_bridge = CvBridge()
        self.yolo_model = YOLO("yolov8x.pt")

        self.iou_buffer = [[] for _ in self.slot_masks]
        self.iou_buffer_size = 50
        for slot in self.iou_buffer:
            slot.extend([None] * self.iou_buffer_size)


    
    def on_parkinglot_cam_image_receive(self, image_msg):
        cv_image = self.cv_bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')

        start = datetime.datetime.now()

        results = self.yolo_model.track(cv_image, persist=True) 

        available_det_boxes = []

        # check result
        for detection_box in results[0].boxes.data.tolist():
            confidence = detection_box[5]

            if float(confidence) < CONFIDENCE_THRESHOLD:
                continue

            if detection_box[6] not in [2, 7]:
                continue

            xmin, ymin, xmax, ymax = int(detection_box[0]), int(detection_box[1]), int(detection_box[2]), int(detection_box[3])
            cv2.rectangle(cv_image, (xmin, ymin) , (xmax, ymax), BLUE, 2)

            available_det_boxes.append(detection_box)

        # check slot
        for mask in self.slot_masks:
            id = mask['slot_id']
            slot_number = mask['slot_number']

            iou, occupied_object_id = check_occupation_for_slot(mask, available_det_boxes)

            self.iou_buffer[id-1].pop(0)
            self.iou_buffer[id-1].append(iou)
            average_iou = 0
            cnt_iou = 0
            for ir in self.iou_buffer[id-1]:
                if ir != None:
                    cnt_iou += 1 
                    average_iou += ir
            average_iou = average_iou / cnt_iou

            points = mask['points']
            pts = np.array(points, np.int32)
            mask_llpt = find_lowest_left_point(pts)
            

            # Draw the poly on the image
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.5
            font_thickness = 2
            font_color = ORANGE
            cv2.putText(cv_image, f"id:{id}", (int(mask_llpt[0]), int(mask_llpt[1])+20), font, font_scale, font_color, font_thickness)
            cv2.putText(cv_image, f"iou:{round(average_iou*100,2)}%", (int(mask_llpt[0]), int(mask_llpt[1])+40), font, font_scale, font_color, font_thickness)

            pts = pts.reshape((-1, 1, 2))

            pslot_stt_msg = String()
            if average_iou > 0.2:
                if occupied_object_id in [2,7]:
                    cv2.polylines(cv_image, [pts], isClosed=True, color=RED, thickness=font_thickness)
                    cv2.putText(cv_image, f"occupied", (int(mask_llpt[0])+50, int(mask_llpt[1])+20), font, font_scale, RED, font_thickness)

                    pslot_stt_msg.data = f"{slot_number},occupied"
                else:
                    pslot_stt_msg.data = f"{slot_number},free" #TODO: handle not car occupying
            else:
                cv2.polylines(cv_image, [pts], isClosed=True, color=GREEN, thickness=font_thickness)
                cv2.putText(cv_image, f"free", (int(mask_llpt[0])+50, int(mask_llpt[1])+20), font, font_scale, GREEN, font_thickness)

                pslot_stt_msg.data = f"{slot_number},free"
            
            self.pslot_stt_pub.publish(pslot_stt_msg)
        

        # end time to compute the fps
        end = datetime.datetime.now()
        # show the time it took to process 1 frame 
        total = (end - start).total_seconds()
        self.get_logger().info(f"Time to process 1 frame: {total * 1000:.0f} milliseconds")

    
        # calculate the frame per second and draw it on the frame
        fps = f"FPS: {1 / total:.2f}"
        cv2.putText(cv_image, fps, (50, 50),
                cv2.FONT_HERSHEY_SIMPLEX, 2, RED, 8)


   
        cv2.imshow("Frame", cv2.resize(cv_image, (1280, 720)))
        cv2.waitKey(1)



def main(args=None):
    rclpy.init(args=args)
    pssd_node = PSlotStatusDetector()
    rclpy.spin(pssd_node)
    pssd_node.destroy_node()
    rclpy.shutdown()