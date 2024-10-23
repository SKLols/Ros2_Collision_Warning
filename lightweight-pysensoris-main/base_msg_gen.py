import random
from sensoris_coding import SensorisHandler
import rclpy
from first_mile_msgs.msg import TrackObjectList
from first_mile_msgs.msg import TrackObject
from queue import Queue
import time
import threading
import numpy as np

import argparse

def arg_parse():
    parser = argparse.ArgumentParser(description="Dummy Ros Data Generator.")

    parser.add_argument('-tn', "--topic-name", required=True, type=str,
                        help='topic name, for subscribing the sensor data, please make sure the topic name in '
                             'configuration file should be changed accordingly')

    parser.add_argument('-nn', "--node-name", required=True, type=str,
                        help='ros node name, cannot be the same as other existing ros nodes')

    args = parser.parse_args()

    return args

class BaseMsgGenerator(object):
    def __init__(self, freq):
        self.q = Queue()
        self.freq = freq

    def gen_random_val(self, param_type, upper, lower, dig=6):
        val = None
        if param_type == "int":
            val = random.uniform(lower, upper)
            val = round(val)
        elif param_type == "float":
            val = round(random.uniform(lower, upper),6)
            val = val
        else:
            print("Parameter type is not supported.")
        return val

    def start(self):
        """
        Start the client thread
        :param q: Local queue including to-be-sent SENSORIS data
        """
        t_gen_test_msg = threading.Thread(target=self.gen_test_msg, args=())
        t_gen_test_msg.daemon = True
        t_gen_test_msg.start()
        # t_sed_sensoris_msg.join()

    def gen_test_msg(self):


            while True:
                time.sleep(1/self.freq)
                ros_msg = TrackObjectList()
                # sensor source
                ros_msg.mast_id = random.choice(["mast_12", "mast_13", "mast_14", "mast_15", "mast_16", "mast_17", "mast_18", "mast_19", "mast_20", "mast_21", "mast_22"])
                ros_msg.sensor_id = random.choice(["local_fusion", "camera", "lidar", "cpm", "global_fusion"])
                ros_msg.sensor_id = random.choice(["global_fusion"])
                if ros_msg.sensor_id == "cpm":
                    ros_msg.station_id = "1234567890"
                else:
                    ros_msg.station_id = "unknown"

                # frame_id out of header or seq in header, frame_id in header is not used.
                ros_msg.frame_id = self.gen_random_val("int", 1, 5)
                #ros_msg.header.seq = np.uint32(self.gen_random_val("int", 1, 5))

                # timestampe in header, nsecs is not used
                now = rclpy.get_rostime()
                ros_msg.header.stamp.secs = now.secs
                ros_msg.header.stamp.nsecs = now.nsecs

                # enumerate the object in object list
                n_objs = self.gen_random_val("int", 1, 5)
                for obj_id in range(n_objs):
                    # initialize a "TrackedObject" based on ros data structure
                    tracked_object = TrackObject()

                    # If it is failed while parsing, then: ret_code = -1, obj_list = None

                    # tracking id
                    tracked_object.object_id = self.gen_random_val("int", 0, 100)

                    tracked_object.box_length_x = self.gen_random_val("float", 1, 5)
                    tracked_object.box_length_y = self.gen_random_val("float", 1, 5)
                    tracked_object.box_length_z = self.gen_random_val("float", 1, 5)

                    if ((ros_msg.sensor_id == "cpm" and tracked_object.object_id == 0)
                     or ros_msg.sensor_id == "global_fusion"):
                        # position of latitude, longitude, attitude
                        tracked_object.latitude = self.gen_random_val("float", 48.754488, 48.754883, 6)
                        print(tracked_object.latitude)
                        tracked_object.longitude = self.gen_random_val("float", 11.477175, 11.47818, 6)
                        tracked_object.altitude = self.gen_random_val("float", 1, 5)
                    else:
                        # position in x, y, z
                        tracked_object.pose_x = self.gen_random_val("float", 50, 50)
                        tracked_object.pose_y = self.gen_random_val("float", 50, 50)
                        tracked_object.pose_z = self.gen_random_val("float", 1, 5)

                    # orientation of yaw, pitch, row
                    tracked_object.orientation_z = self.gen_random_val("float", 1, 5)
                    tracked_object.orientation_y = self.gen_random_val("float", 1, 5)
                    tracked_object.orientation_x = self.gen_random_val("float", 1, 5)

                    # velocity in x, y, z
                    tracked_object.vel_x = self.gen_random_val("float", 1, 5)
                    tracked_object.vel_y = self.gen_random_val("float", 1, 5)
                    tracked_object.vel_z = self.gen_random_val("float", 1, 5)

                    # acceleration in x, y, z
                    tracked_object.acc_x = self.gen_random_val("float", 1, 2)
                    tracked_object.acc_y = self.gen_random_val("float", 1, 2)
                    tracked_object.acc_z = self.gen_random_val("float", 1, 2)

                    # object type

                    tracked_object.object_class = random.choice(["car", 'bus', 'truck', 'motorcycle', 'bicycle', 'person'])

                    # classification confidence
                    tracked_object.classification_confidence = self.gen_random_val("float", 1, 5)

                    # detection confidence
                    tracked_object.detection_confidence  = self.gen_random_val("float", 1, 5)

                    # covariance, which has not be specified yet
                    tracked_object.covariance = [2.33, 2.33, 3.11, 4.11, 4.21, 4.21, 1.45, 12.34, 34.5323, 12.3123, 543.1321, 123.421, 123.4, 123.1323, 324.123, 354.13]

                    ros_msg.object_list.append(tracked_object)

                # number of object
                ros_msg.total_objects = len(ros_msg.object_list)
                print("Generated a ros message")
                self.q.put(ros_msg)

    def ros_publish(self, topic_name, node_name):
        sh = SensorisHandler()
        pub = rclpy.Publisher(topic_name, TrackObjectList, queue_size=524288)
        rclpy.init_node(node_name, anonymous=True)
        self.start()
        rate = rclpy.Rate(self.freq)  # 10hz
        while not rclpy.is_shutdown():
            if not self.q.empty():
                ros_msg = self.q.get()
                pub.publish(ros_msg)
        rclpy.spin()


if __name__ == "__main__":
    args = arg_parse()
    bmg = BaseMsgGenerator(1000)
    print("start generating")
    bmg.ros_publish(topic_name=args.topic_name, node_name=args.node_name)
    
    
