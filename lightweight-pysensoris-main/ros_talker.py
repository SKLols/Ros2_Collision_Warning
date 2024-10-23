# !/usr/bin/env python
from queue import Queue
import rclpy
from first_mile_msgs.msg import TrackObjectList
from sensoris_coding import SensorisHandler


def run(sensor_interface_name: str, q: Queue, rostopic: str):
    sh = SensorisHandler()
    pub = rclpy.Publisher(rostopic, TrackObjectList, queue_size=524288)
    rclpy.init_node(sensor_interface_name, anonymous=True)
    rate = rclpy.Rate(10)  # 10hz
    while not rclpy.is_shutdown():
        if not q.empty():
            data = q.get()
            ret_code, ros_msg = sh.decode(data)
            print(ros_msg)
            pub.publish(ros_msg)