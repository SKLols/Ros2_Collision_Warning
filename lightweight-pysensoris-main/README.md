# lightweight-SensorTraffix

This is a lightweight version for SensorTraffix (SENSORIS-based Sensor Traffic Data Exchange) implementation in python. 

It is aimed to accelerate the message building by considering only the minimum of the requirements on data fields.

Compared to the full SensorTraffix, lightweight-SensorTraffix can reduce the message handling time from >20 ms to <1 ms.

Additionally, the installation steps are optimized as one-button-press.

## Installation

* Step 1: Navigate to src folder in ros workspace: run `cd /path/to/ros-workspace/src`
* Step 2: Clone the git repository in src folder: run `git clone https://gitlab-extern.ivi.fraunhofer.de/rsong/lightweight-pysensoris.git`
* Step 3: Install the required python packages: run `pip3 install -r requirements.txt`. 
* ~~Step 4 (optional): Download the sensoris v1.3.1 is downloaded from official website and complie into this project: run `python3 sensoris_prep.py`~~
* Step 5: Navigate to ros workspace: run `cd /path/to/ros-workspace` 
* Step 6: Update the ros project: run `catkin_make --pkg in2lab_msgs`
* Step 7: Navigate to SensorTraffix project: run `cd ./src/lightweight-pysensoris`
* Step 8: Active the ros environment: run `source ../../setup.bash`

Note that the Step 4 should be skipped when running the software package for first mile test filed!!


## Version

 * SENSORIS Version: [v1.3.1 public](https://sensoris.org/wp-content/uploads/sites/21/2022/04/sensoris-specification-v1.3.1-1.zip) 
 * Protobuf Version: v3.19.0 or v3.20.0

## Execution

### To start sensoris client:
* Step 9a: run `python3 main_sending.py [args]`. (i) The ros node is created, which subscribes ros-msg from a specific topic; (ii) The sensoris-server is created, which sends out sensoris-msg.

* Run  `python3 main_sending.py -h` to get the help text:
  
```
usage: main_sending.py [-h] -ip TARGET_IP -pt PORT_NUMBER -tn TOPIC_NAME -nn
                       NODE_NAME -mt MQTT_TOPIC_NAME

Sensoris Data Transmission.

optional arguments:
  -h, --help            show this help message and exit
  -ip TARGET_IP, --target-ip TARGET_IP
                        Target IP address
  -pt PORT_NUMBER, --port-number PORT_NUMBER
                        Target IP address
  -tn TOPIC_NAME, --topic-name TOPIC_NAME
                        topic name, for subscribing the sensor data, please
                        make sure the topic name in configuration file should
                        be changed accordingly
  -nn NODE_NAME, --node-name NODE_NAME
                        ros node name, cannot be the same as other existing
                        ros nodes
  -mt MQTT_TOPIC_NAME, --mqtt-topic MQTT_TOPIC_NAME
                        mqtt topic name, distinated mqtt topic for sensoirs 
                        message sending.

 ```
*  Example: `python3 main_sending.py -ip 127.0.0.1 -pt 10001 -tn mast_5/fused_tracked_objects -nn test_listener -mt "mqtt_test_topic"`
*  Example for 5GoIng: `python3 main_sending.py -ip 192.168.199.80 -pt 1883 -tn mast_22/fused_tracked_objects -nn sensoris_listener_22 -mt sensoris_5GoIng/mqtt/mast_23/fused_tracked_objects`
 


### To start sensoris server:
* Step 9b: run `python3 main_reciving.py [args]`. (i) The sensoris-server is created, which receives sensoris-msg; (ii) The ros node is created publishes ros msg to a specific topic.

* Run  `python3 main_receiving.py -h` to get the help text:

```
usage: main_receiving.py [-h] -ip SERVER_IP -pt PORT_NUMBER -tn TOPIC_NAME -nn
                         NODE_NAME

Sensoris Data Transmission.

optional arguments:
  -h, --help            show this help message and exit
  -ip SERVER_IP, --server-ip SERVER_IP
                        Target IP address
  -pt PORT_NUMBER, --port-number PORT_NUMBER
                        Target IP address
  -tn TOPIC_NAME, --topic-name TOPIC_NAME
                        topic name, for subscribing the sensor data, please
                        make sure the topic name in configuration file should
                        be changed accordingly
  -nn NODE_NAME, --node-name NODE_NAME
                        ros node name, cannot be the same as other existing
                        ros nodes
```


## Docker Container (testing...)
* Step 1: Navigate to SensorTraffix project: run `cd ./src/lightweight-pysensoris`
* Step 2: Make container: run `docker build --tag python-docker`
* Step 3: Run Docker: run `docker run python-docker`

## Remarks
* SensorTraffix should be started after sensor fusion activated.

