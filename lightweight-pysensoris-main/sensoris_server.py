#!/usr/bin/env python
import socket
import threading
from sensoris.protobuf.messages.data_pb2 import DataMessages
from queue import Queue
import sys
from datetime import datetime
from datetime import date
import os

from sensoris.protobuf.types.base_pb2 import Int64Value

import json
from json import dumps, loads, JSONEncoder, JSONDecoder
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


class SensorisServer(object):
    def __init__(self, server_ip='192.168.100.10', port=9999):
        self.server_ip = server_ip
        self.port = port
        self.buffer_size = 524288

    def start(self, q: Queue):
        """
        Start the client thread
        :param q: Local queue including to-be-received SENSORIS data
        """
        t_rec_sensoris_msg = threading.Thread(target=self.receive_sensoris_data_message, args=(self.server_ip, self.port, q))
        t_rec_sensoris_msg.daemon = True
        t_rec_sensoris_msg.start()
        print("Server @Port" + str(self.port) + " is listening the mObject data...")

    def receive_sensoris_data_message(self, server_ip: str, port: int,  q: Queue):
        """
        The main process of SENSORIS server
        :param port: Port number in the server
        :param server_ip: IP address in the server
        :param q: Local queue including to-be-received SENSORIS data
        """
        rec_sensoris_msg = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        rec_sensoris_msg.bind((server_ip, port))

        try:
            while True:

                data, msg_address = rec_sensoris_msg.recvfrom(self.buffer_size)
                print("Client@" + str(port) + ": get the SENSORIS Data Message (size=" + str(
                    sys.getsizeof(data)) + " bytes)")
                ####
                if not os.path.exists('sensoris_msg'):
                    os.makedirs('sensoris_msg')
                today = date.today()
                now = datetime.now().time()
                current_time = today.strftime("%Y%m%d_") + now.strftime("%H%M%S")
                with open("sensoris_msg/test_%s.txt" % current_time, "wb") as binary_file:
                    # Write bytes to file
                    binary_file.write(data)
                ####
                q.put(data)

        except KeyboardInterrupt:
            rec_sensoris_msg.close()


def run(q: Queue, server_ip, port):
    sensoris_server = SensorisServer(server_ip=server_ip, port=port)
    sensoris_server.start(q)


if __name__ == '__main__':
    q = Queue()
    sensoris_server = SensorisServer(server_ip='127.0.0.1', port=10000)
    sensoris_server.start(q)


