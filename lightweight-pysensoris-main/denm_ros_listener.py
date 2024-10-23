#!/usr/bin/env python3
from queue import Queue
import rclpy
from denm_sensoris_coding import SensorisHandler
from etsi_its_msgs.msg import DENM
import time
import datetime
import json
from json import JSONEncoder
import pickle

json_types = (list, dict, str, int, float, bool, type(None))


class PythonObjectEncoder(JSONEncoder):
    def default(self, obj):
        if isinstance(obj, json_types):
            return super().default(self, obj)
        return {'_python_object': pickle.dumps(obj).decode('latin-1')}


def as_python_object(dct):
    if '_python_object' in dct:
        return pickle.loads(dct['_python_object'].encode('latin-1'))
    return dct


def callback_denm(data, args):
    # Parse the arguments
    se = args[0]
    q = args[1]
    sample = args[2]
    # start_time = time.time()
    sensoris_bytes = se.encode(data)
    # print("--- %s seconds ---" % (time.time() - start_time))

    if sample is not None:
        print("Writing file")
        sample["sensoris_binary"].append(sensoris_bytes)
        with open(sample["Date & Time"] + "_sample.json", "w") as outfile:
            json.dump(sample, outfile, cls=PythonObjectEncoder)
    q.put(sensoris_bytes)


def run(sensor_interface_name: str, q: Queue, rostopic: str, save_bin=0):

    sh = SensorisHandler()
    rclpy.init_node(sensor_interface_name)
    print(rostopic)
    sample = None
    if save_bin:
        dt_now = datetime.datetime.now()
        sample = {"Information": "This is a sample JSON file for sensoris messages in binary format \n A sequential sensoris binary messages are saved in the list of 'sensoris_binary'",
                  "Date & Time": str(dt_now),
                  "Contact": "rui.song@ivi.fraunhofer.de",
                  "sensoris_binary":[]}
    rclpy.Subscriber(rostopic,
                     DENM,
                     callback_denm,
                     (sh, q, sample))
    rclpy.spin()



