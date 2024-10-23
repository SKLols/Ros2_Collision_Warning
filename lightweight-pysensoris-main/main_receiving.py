# -*- coding: utf-8 -*-
"""
Script to start the sensoris data transmission
"""

# Author: Rui Song <rui.song@ivi.fraunhofer.de>
# License: to be defined

import argparse
import importlib
import os
import sys
from queue import Queue


def arg_parse():
    parser = argparse.ArgumentParser(description="Sensoris Data Transmission.")
    parser.add_argument('-ip', "--server-ip", required=True, type=str,
                        help='Target IP address')

    parser.add_argument('-pt', "--port-number", required=True, type=int,
                        help='Target IP address')

    parser.add_argument('-tn', "--topic-name", required=True, type=str,
                        help='topic name, for subscribing the sensor data, please make sure the topic name in '
                             'configuration file should be changed accordingly')

    parser.add_argument('-nn', "--node-name", required=True, type=str,
                        help='ros node name, cannot be the same as other existing ros nodes')

    args = parser.parse_args()

    return args


def main():
    args = arg_parse()

    ros_talker = importlib.import_module("ros_talker")
    sensoris_server = importlib.import_module("sensoris_server")

    q = Queue()

    server = getattr(sensoris_server, "run")
    server(q, server_ip=args.server_ip, port=args.port_number)

    talker = getattr(ros_talker, "run")
    talker(args.node_name, q, args.topic_name)


if __name__ == '__main__':
    main()
