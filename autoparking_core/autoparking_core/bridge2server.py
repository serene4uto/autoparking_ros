import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import os
from ament_index_python.packages import get_package_share_directory

import firebase_admin
from firebase_admin import credentials
from firebase_admin import firestore

class Bridge2Server(Node):
    def __init__(self):
        super().__init__("bridge_2_server")

        # Get the path to the package's 'share' directory
        package_share_directory = get_package_share_directory('autoparking_core')

        # Construct the full path to your file
        firebase_creds_file_path = os.path.join(package_share_directory, 'autoparking-project-firebase-adminsdk-dldvj-5b4a6edb39.json')
        
        self.fb_cred = credentials.Certificate(firebase_creds_file_path)
        self.fb_default_app = firebase_admin.initialize_app(self.fb_cred)
        self.fb_db = firestore.client()

        self.ref_message = self.fb_db.collection(u'message')
        self.ref_vehicle = self.fb_db.collection(u'vehicle')
        self.ref_parking_slot = self.fb_db.collection(u'parking_slot')
        

        self.vcmd_pub = self.create_publisher(String, "aps/vehicle_cmd", 10)
        self.vstt_sub = self.create_subscription(String, "aps/vehicle_status", self.on_vehicle_stt_receive, 10)
        self.psstt_sub = self.create_subscription(String, "aps/parking_slot_status", self.on_parking_slot_stt_receive, 10)


        self.slot_dict = {}
        self.vehicle_dict = {}

        # old_docs = self.ref_message.stream()
        # for doc in old_docs:
        #     doc.reference.delete()
            # self.ref_message.document(doc.id).delete()

        self.ref_message.on_snapshot(self.on_fbstore_message_receive)


    def on_fbstore_message_receive(self, doc_snapshot, changes, read_time):
        for change in changes:
            # if change.type.name == 'ADDED':

            if change.document.to_dict()['status'] != 'unseen':
                break

            vcmd_msg = String()
            vcmd_msg.data = f"{change.document.to_dict()['text']}"
            self.vcmd_pub.publish(vcmd_msg)


            self.get_logger().info(f"New document added: {change.document.id}")
            for field, value in change.document.to_dict().items():
                self.get_logger().info(f"{field}: {value}")

            updated_data = {
                "status": "seen"
            }

            self.ref_message.document(change.document.id).update(updated_data)


            # elif change.type.name == 'MODIFIED':
            #     print(f"Document modified: {change.document.id}")
            # elif change.type.name == 'REMOVED':
            #     print(f"Document removed: {change.document.id}")
        

    def on_vehicle_stt_receive(self, vstt_msg):
        license_plate, vstatus, current_slot = vstt_msg.data.split(',')

        existing_value = self.slot_dict.get(license_plate)
        if existing_value == f"{vstatus},{current_slot}":
            # self.get_logger().info(f"vehicle stt same")
            return
        # else:
        #     self.get_logger().info(f"new vehicle stt {license_plate} {vstatus} {current_slot}")
        
        self.slot_dict[license_plate] = f"{vstatus},{current_slot}"

        # Query for the document with the given license_plate
        vehicle_query = self.ref_vehicle.where("license_plate", "==", license_plate)

        # Execute the query to get the document snapshot
        vdocs = vehicle_query.stream()

        vehicle_id = None
        # Update the document if it exists
        for doc in vdocs:
            doc_ref = self.ref_vehicle.document(doc.id)
            vehicle_id = doc.id
            doc_ref.update({"status": vstatus,
                            "parking_slot": current_slot})
            
        if (vstatus == "ready-to-park") or (vstatus == "not-parked"):

            psquery = self.ref_parking_slot.where("vehicle_id", "==", vehicle_id)
            psdoc = psquery.stream()

            for doc in psdoc:
                doc_ref = self.ref_parking_slot.document(doc.id)
                doc_ref.update({"vehicle_id": ""})

        else:

            psquery = self.ref_parking_slot.where("slot_name", "==", current_slot)
            psdoc = psquery.stream()

            for doc in psdoc:
                doc_ref = self.ref_parking_slot.document(doc.id)
                doc_ref.update({"vehicle_id": vehicle_id})


    def on_parking_slot_stt_receive(self, psstt_msg):
        slot_name, new_status = psstt_msg.data.split(',')

        # Use dict.get() to retrieve the existing value (default to None if the key doesn't exist)
        existing_value = self.slot_dict.get(slot_name)

        # self.get_logger().info(existing_value)

        # Compare the existing value with the new value and update the dictionary if different
        if existing_value != new_status:
            self.slot_dict[slot_name] = new_status

            # Handle Firestore update
            try:
                query = self.ref_parking_slot.where("slot_name", "==", slot_name)
                docs = query.stream()

                # Update the document if it exists
                for doc in docs:
                    doc_ref = self.ref_parking_slot.document(doc.id)
                    doc_ref.update({"status": new_status})

                self.get_logger().info(f"Public pstt {slot_name}: {new_status}")
            except Exception as e:
                self.get_logger().error(f"Error updating Firestore: {e}")

            


def main(args=None):
    rclpy.init(args=args)
    b2s_node = Bridge2Server()
    rclpy.spin(b2s_node)
    b2s_node.destroy_node()
    rclpy.shutdown()
