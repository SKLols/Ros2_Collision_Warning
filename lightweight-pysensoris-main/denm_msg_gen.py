import random
import rclpy
from rclpy.node import Node
from ros_etsi_its_msgs.msg import DENM
from queue import Queue
import time
import threading
import argparse
import sys

def arg_parse():
    parser = argparse.ArgumentParser(description="Dummy ROS 2 Data Generator.")
    parser.add_argument('-tn', "--topic-name", required=True, type=str,
                        help='Topic name for publishing the sensor data. Please make sure the topic name in '
                             'configuration file should be changed accordingly.')
    parser.add_argument('-nn', "--node-name", required=True, type=str,
                        help='ROS node name, cannot be the same as other existing ROS nodes.')
    args = parser.parse_args()
    return args

class DENMMsgGenerator(Node):
    def __init__(self, node_name, topic_name, freq):
        super().__init__(node_name)
        self.q = Queue()
        self.freq = freq
        self.publisher_ = self.create_publisher(DENM, topic_name, 10)
        self.create_timer(1.0 / freq, self.ros_publish)
        self.start()

    def gen_random_val(self, param_type, upper, lower, dig=6):
        if param_type == "int":
            val = random.randint(lower, upper)
        elif param_type == "float":
            val = round(random.uniform(lower, upper), dig)
        else:
            self.get_logger().error("Parameter type is not supported.")
            val = None
        return val

    def start(self):
        t_gen_test_msg = threading.Thread(target=self.gen_test_msg)
        t_gen_test_msg.daemon = True
        t_gen_test_msg.start()

    def gen_test_msg(self):
        while rclpy.ok():
            time.sleep(1 / self.freq)
            ros_denm_msg = DENM()
            ros_denm_msg.header.stamp = self.get_clock().now().to_msg()
            ros_denm_msg.header.frame_id = "denm_frame"

            ros_denm_msg.its_header.protocol_version = 6
            ros_denm_msg.its_header.message_id = 7
            ros_denm_msg.its_header.station_id = 8

            ros_denm_msg.management.event_position.latitude = 487679921
            ros_denm_msg.management.event_position.longitude = 114370772

            ros_denm_msg.has_situation = True
            ros_denm_msg.situation.has_linked_cause = True
            ros_denm_msg.situation.linked_cause.cause_code = 9
            ros_denm_msg.situation.linked_cause.sub_cause_code = 5
            ros_denm_msg.situation.event_type.cause_code = 19
            ros_denm_msg.situation.event_type.sub_cause_code = 15

            self.q.put(ros_denm_msg)
            self.get_logger().info("Generated and added message to queue")

    def ros_publish(self):
        if not self.q.empty():
            ros_msg = self.q.get()
            self.publisher_.publish(ros_msg)
            self.get_logger().info("Published a ROS 2 message")

def main():
    args = arg_parse()
    rclpy.init(args=sys.argv)  # Use sys.argv here

    denm_msg_generator = DENMMsgGenerator(node_name=args.node_name, topic_name=args.topic_name, freq=1)

    try:
        rclpy.spin(denm_msg_generator)
    except KeyboardInterrupt:
        denm_msg_generator.get_logger().info('Keyboard interrupt detected, shutting down gracefully...')
    finally:
        denm_msg_generator.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
