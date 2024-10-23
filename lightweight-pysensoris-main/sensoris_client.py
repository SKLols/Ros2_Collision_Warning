# -*- coding: utf-8 -*-
import socket
import time
import numpy as np
from sensoris.protobuf.messages.data_pb2 import DataMessages
import threading
from queue import Queue
import sys
import time
import paho.mqtt.client as paho
from paho import mqtt


class SensorisClient(object):

    def __init__(self, target_ip, port, mqtt_topic):
        # 192.168.199.197
        self.target_ip = target_ip
        self.buffer_size = 99999
        self.port = port
        self.mqtt_topic = mqtt_topic

    def start_sed(self, q):
        """
        Start the client thread
        :param q: Local queue including to-be-sent SENSORIS data
        """
        t_sed_sensoris_msg = threading.Thread(target=self.send_data_message, args=(self.target_ip, self.port, q, self.mqtt_topic))
        t_sed_sensoris_msg.daemon = True
        t_sed_sensoris_msg.start()
        # t_sed_sensoris_msg.join()

    @staticmethod
    def send_data_message(target_ip, port, q: Queue, mqtt_topic="sensoris_5GoIng/mqtt/mast_23/fused_tracked_objects"):
        """
        The main process of SENSORIS client
        :param target_ip: Target IP address
        :param port: The port number for connection
        :param q: Local queue including to-be-sent SENSORIS data
        """
        def on_connect(self, mosq, obj, rc, test):
            print('on connect...')
            print('publish messages on mqtt topic: ' + mqtt_topic)
        client = paho.Client(client_id="", userdata=None, protocol=paho.MQTTv5)
        client.on_connect = on_connect
       
            
        try:
            client.connect(target_ip, port, 9999)
        except:
            print("connection failed")
            client.disconnect()
            exit(1)
        #client.on_connect = on_connect
        #send_sensoris_msg = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        timeout = time.time() + 1

        try:
            while True:

                if q.empty():
                    if time.time() < timeout:
                        continue
                    print("Client@:" + str(port) + ": Sent the heart beat")
                    data_messages = DataMessages()
                    data = data_messages.SerializeToString()
                    #send_sensoris_msg.sendto(data, (target_ip, port))
                    client.publish(mqtt_topic, data, qos=0)
                    timeout = time.time() + 2
                else:
                    data = q.get()
                    client.publish(mqtt_topic, data, qos=0)
                    #send_sensoris_msg.sendto(data, (target_ip, port))
                    timeout = time.time() + 2
                    print("Client@" + str(port) + ": Sent the SENSORIS Data Message (size=" + str(
                        sys.getsizeof(data)) + " bytes)")
                
        except KeyboardInterrupt:
            client.disconnect()

    def start_rec(self, q):
        """
        Start the client thread
        :param q: Local queue including to-be-sent SENSORIS data
        """
        t_rec_sensoris_msg = threading.Thread(target=self.receive_data_message, args=(self.target_ip, self.port, q, self.mqtt_topic))
        t_rec_sensoris_msg.daemon = True
        t_rec_sensoris_msg.start()
        # t_sed_sensoris_msg.join()

    def receive_data_message(self, target_ip, port, q: Queue, mqtt_topic="sensoris_5GoIng/mqtt/global_objects"):
        """
        The main process of SENSORIS server
        :param port: Port number in the server
        :param server_ip: IP address in the server
        :param q: Local queue including to-be-received SENSORIS data
        """
        def on_connect(self, mosq, obj, rc, test):
            print('on connect...')
            print('subscribe the mqtt topic: ' + mqtt_topic)
            self.subscribe(mqtt_topic)

        def on_message(self, userdata, message):
            print(message.payload)
            userdata.put(message.payload)

        client = paho.Client(client_id="", userdata=q, protocol=paho.MQTTv5)
        client.on_connect = on_connect
        try:
            client.connect(target_ip, port, 9999)
        except:
            print("connection failed")
            client.disconnect()
            exit(1)
        
        client.on_message = on_message
        client.loop_forever()

'''
        try:
            while True:
                time.sleep(1)
                #client.on_message()
                client.subscribe("sensoris_5GoIng/data_object/binary", qos=1)
                client.on_message = on_message
                
                #print(data)
                #q.put(data)

        except KeyboardInterrupt:
            client.disconnect()
'''

def run_sed(q: Queue, target_ip, port, mqtt_topic="sensoris_5GoIng/mqtt/mast_23/fused_tracked_objects"):
    sensoris_client = SensorisClient(target_ip=target_ip, port=port, mqtt_topic=mqtt_topic)
    sensoris_client.start_sed(q)

def run_rec(q: Queue, target_ip, port, mqtt_topic="sensoris_5GoIng/mqtt/global_objects"):
    sensoris_client = SensorisClient(target_ip=target_ip, port=port, mqtt_topic=mqtt_topic)
    sensoris_client.start_rec(q)


if __name__ == '__main__':
    q = Queue()
    sensoris_client = SensorisClient(target_ip='127.0.0.1', port=10000)
    sensoris_client.start_send(q)
