from sensoris.protobuf.messages.data_pb2 import DataMessages, DataMessage
from sensoris.protobuf.messages.data_pb2 import EventGroup
from sensoris.protobuf.categories.traffic_events_pb2 import Hazard
from etsi_its_msgs.msg import DENM
from google.protobuf.wrappers_pb2 import Int64Value


class SensorisHandler(object):

    object_type_matching = {
        '': -1,
        'Object': 0,  # Unknown
        'car': 1,
        'bus': 4,
        'truck': 3,
        'motorcycle': 8,
        'bicycle': 9,
        'person': 10,
        -1: '',
        0: 'Object',
        1: 'car',
        4: 'bus',
        3: 'truck',
        8: 'motorcycle',
        9: 'bicycle',
        10: 'person'
    }

    # map the mast id from sensoris to ros
    mast_id_matching = {
        "unknown": -1,
        "mast_01": 1,
        "mast_02": 2,
        "mast_03": 3,
        "mast_04": 4,
        "mast_05": 5,
        "mast_06": 6,
        "mast_07": 7,
        "mast_08": 8,
        "mast_09": 9,
        "mast_10": 10,
        "mast_11": 11,
        "mast_12": 12,
        "mast_13": 13,
        "mast_14": 14,
        "mast_15": 15,
        "mast_16": 16,
        "mast_17": 17,
        "mast_18": 18,
        "mast_19": 19,
        "mast_20": 20,
        "mast_21": 21,
        "mast_22": 22,
        -1: "Unknown",
        1: "mast_01",
        2: "mast_02",
        3: "mast_03",
        4: "mast_04",
        5: "mast_05",
        6: "mast_06",
        7: "mast_07",
        8: "mast_08",
        9: "mast_09",
        10: "mast_10",
        11: "mast_11",
        12: "mast_12",
        13: "mast_13",
        14: "mast_14",
        15: "mast_15",
        16: "mast_16",
        17: "mast_17",
        18: "mast_18",
        19: "mast_19",
        20: "mast_20",
        21: "mast_21",
        22: "mast_22",
    }

    # map the sensor id from sensoris to ros
    sensor_id_matching = {
        "unknown": -1,
        "local_fusion": 1,
        "global_fusion": 2,
        "camera": 3,
        "lidar": 4,
        "cpm": 5,
        "cam": 6,
        "denm": 7,
        -1: "Unknown",
        1: "local_fusion",
        2: "global_fusion",
        3: "camera",
        4: "lidar",
        5: "cpm",
        6: "cam",
        7: "denm",
    }


    def __init__(self, print_logs=0, config_dict=dict({})):
        # will not be used
        self.config = config_dict
        self.print_logs = print_logs

    def encode(self, ros_msg: DENM):

        data_message = DataMessage()

        event_source = DataMessage.EventSource()
        event_source.source.id.value = -1
        data_message.event_source.append(event_source)

        event_source = DataMessage.EventSource()   
        event_source.source.id.value = -1
        data_message.event_source.append(event_source)

        event_source = DataMessage.EventSource() 
        event_source.source.id.value = -1
        data_message.event_source.append(event_source)

        data_message.envelope.ids.message_id.value = 1

        event_group = EventGroup()
        event_group.envelope.origin.timestamp.posix_time.value = int(str(ros_msg.header.stamp.secs) + str(ros_msg.header.stamp.nsecs)[0:6])
        
        hazard = Hazard()
        hazard.type_and_confidence.type = 1
        event_group.traffic_events_category.hazard.append(hazard)

        data_message.event_group.append(event_group)
        sensoris_msg = DataMessages()
        sensoris_msg.data_message.append(data_message)
        #print(sensoris_msg)
        sensoris_bytes = sensoris_msg.SerializeToString()

        return sensoris_bytes

    @staticmethod
    def _check_sensoris_msg(print_logs: int, sensoris_data_msgs: DataMessages) -> int:

        if not sensoris_data_msgs.data_message:
            if print_logs:
                print("SensorisParser: Received Heartbeat, None will be returned")
            ret_code = 0
            return ret_code

        if len(sensoris_data_msgs.data_message) > 1:
            if print_logs:
                print("SensorisParser: Received Sensoris message with non-one sensoris data_message")
            ret_code = -1
            return ret_code

        for data_message in sensoris_data_msgs.data_message:
            if len(data_message.event_group) != 1:
                if print_logs:
                    print("SensorisParser: Received Sensoris message with non-one sensoris event_group")
                ret_code = -1
                return ret_code

        ret_code = 1

        return ret_code

    @staticmethod
    def _float_to_int(value: int, exponent=3) -> float:
        return int(value * (10**exponent))

    @staticmethod
    def _int_to_float(value: int, exponent=3) -> float:
        return value / (10**exponent)

    def trans_object_type(self, object_type):

        if not self.object_type_matching.get(object_type):  # object_type cannot be found in the predefined directory
            return -1
        else:
            return self.object_type_matching[object_type]

    def trans_mast_id(self, mast_id):

        if not self.mast_id_matching.get(mast_id):  # mast_id cannot be found in the predefined directory
            return -1
        else:
            return self.mast_id_matching[mast_id]
    
    def trans_sensor_id(self, sensor_id):

        if not self.sensor_id_matching.get(sensor_id):  # sensor_id cannot be found in the predefined directory
            return -1
        else:
            return self.sensor_id_matching[sensor_id]

    def trans_station_id(self, station_id):
        
        try:
            return int(station_id)
        except ValueError:
            return -1

if __name__ == '__main__':
    data_message = DataMessage()
    event_source = DataMessage.EventSource()
    ids = event_source.ids
    from google.protobuf.wrappers_pb2 import Int64Value
    int64value = Int64Value()
    int64value.value = 1
    ids.id.append(int64value)
    data_message.event_source.append(event_source)
    print(data_message)
    movable_object = MovableObject()
    test = movable_object.rectangular_box_and_accuracy.center_orientation_size.center_position_and_accuracy.geographic_wgs84.longitude.value
    pass

