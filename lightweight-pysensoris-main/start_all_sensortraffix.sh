#!/bin/sh


echo "Start all SensorTraffix threads in 5GoIng backend"

python3 main_mqtt_sub.py -ip 192.168.199.80 -pt 1883 -tn mast_12/fused_tracked_objects -nn sensoris_listener_12 -mt sensoris_5GoIng/mqtt/mast_12/fused_tracked_objects &

python3 main_mqtt_sub.py -ip 192.168.199.80 -pt 1883 -tn mast_13/fused_tracked_objects -nn sensoris_listener_13 -mt sensoris_5GoIng/mqtt/mast_13/fused_tracked_objects &

python3 main_mqtt_sub.py -ip 192.168.199.80 -pt 1883 -tn mast_14/fused_tracked_objects -nn sensoris_listener_14 -mt sensoris_5GoIng/mqtt/mast_14/fused_tracked_objects &

python3 main_mqtt_sub.py -ip 192.168.199.80 -pt 1883 -tn mast_15/fused_tracked_objects -nn sensoris_listener_15 -mt sensoris_5GoIng/mqtt/mast_15/fused_tracked_objects &

python3 main_mqtt_sub.py -ip 192.168.199.80 -pt 1883 -tn mast_16/fused_tracked_objects -nn sensoris_listener_16 -mt sensoris_5GoIng/mqtt/mast_16/fused_tracked_objects &

python3 main_mqtt_sub.py -ip 192.168.199.80 -pt 1883 -tn mast_17/fused_tracked_objects -nn sensoris_listener_17 -mt sensoris_5GoIng/mqtt/mast_17/fused_tracked_objects &

python3 main_mqtt_sub.py -ip 192.168.199.80 -pt 1883 -tn mast_18/fused_tracked_objects -nn sensoris_listener_18 -mt sensoris_5GoIng/mqtt/mast_18/fused_tracked_objects &

python3 main_mqtt_sub.py -ip 192.168.199.80 -pt 1883 -tn mast_19/fused_tracked_objects -nn sensoris_listener_19 -mt sensoris_5GoIng/mqtt/mast_19/fused_tracked_objects &

python3 main_mqtt_sub.py -ip 192.168.199.80 -pt 1883 -tn mast_20/fused_tracked_objects -nn sensoris_listener_20 -mt sensoris_5GoIng/mqtt/mast_20/fused_tracked_objects &

python3 main_mqtt_sub.py -ip 192.168.199.80 -pt 1883 -tn mast_21/fused_tracked_objects -nn sensoris_listener_21 -mt sensoris_5GoIng/mqtt/mast_21/fused_tracked_objects &

python3 main_mqtt_sub.py -ip 192.168.199.80 -pt 1883 -tn mast_22/fused_tracked_objects -nn sensoris_listener_22 -mt sensoris_5GoIng/mqtt/mast_22/fused_tracked_objects &

python3 main_mqtt_sub.py -ip 192.168.199.80 -pt 1883 -tn dummy_denm -nn dummy_denm_node -mt sensoris_5GoIng/mqtt/mast_22/fused_tracked_objects &



