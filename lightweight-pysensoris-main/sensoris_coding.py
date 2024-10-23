from sensoris.protobuf.messages.data_pb2 import DataMessages, DataMessage
from sensoris.protobuf.messages.data_pb2 import EventGroup
from sensoris.protobuf.categories.object_detection_pb2 import MovableObject
#from in2lab_msgs.msg import TrackObject, TrackObjectList
from first_mile_msgs.msg import TrackObject, TrackObjectList
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

    def encode(self, ros_msg: TrackObjectList):

        data_message = DataMessage()

        event_source = DataMessage.EventSource()
        if ros_msg.mast_id in self.mast_id_matching:
            event_source.source.id.value = self.trans_mast_id(ros_msg.mast_id)
        else: 
            event_source.source.id.value = self.trans_mast_id(ros_msg.mast_id)
        data_message.event_source.append(event_source)

        event_source = DataMessage.EventSource()    
        if ros_msg.sensor_id in self.sensor_id_matching:
            event_source.source.id.value = self.trans_sensor_id(ros_msg.sensor_id)
        else: 
            event_source.source.id.value = self.trans_sensor_id(ros_msg.sensor_id)
        data_message.event_source.append(event_source)

        event_source = DataMessage.EventSource() 
        event_source.source.id.value = self.trans_station_id(ros_msg.station_id)
        data_message.event_source.append(event_source)

        data_message.envelope.ids.message_id.value = ros_msg.frame_id
     

        event_group = EventGroup()
        event_group.envelope.origin.timestamp.posix_time.value = int(str(ros_msg.header.stamp.secs) + str(ros_msg.header.stamp.nsecs)[0:6])

        #ros_msg.total_objects
        for trj_obj in ros_msg.object_list:
            movable_object = MovableObject()
            movable_object.object_id.value = trj_obj.object_id


            # boundingbox in x, y, z
            movable_object.rectangular_box_and_accuracy.center_orientation_size.size_and_accuracy.metric_vehicle.x.value = self._float_to_int(trj_obj.box_length_x)
            movable_object.rectangular_box_and_accuracy.center_orientation_size.size_and_accuracy.metric_vehicle.y.value = self._float_to_int(trj_obj.box_length_y)
            movable_object.rectangular_box_and_accuracy.center_orientation_size.size_and_accuracy.metric_vehicle.z.value = self._float_to_int(trj_obj.box_length_z)

            if ((data_message.event_source[1].source.id.value == 5 and movable_object.object_id.value == 0)
                or data_message.event_source[1].source.id.value == 2):
                # position in longitude, latitude, attitude
                movable_object.rectangular_box_and_accuracy.center_orientation_size.center_position_and_accuracy.geographic_wgs84.longitude.value = self._float_to_int(trj_obj.longitude, 8)
                movable_object.rectangular_box_and_accuracy.center_orientation_size.center_position_and_accuracy.geographic_wgs84.latitude.value = self._float_to_int(trj_obj.latitude, 8)
                movable_object.rectangular_box_and_accuracy.center_orientation_size.center_position_and_accuracy.geographic_wgs84.altitude.value = self._float_to_int(trj_obj.altitude, 8)
            else:
                # position in x, y, z
                movable_object.rectangular_box_and_accuracy.center_orientation_size.center_position_and_accuracy.metric_vehicle.x.value = self._float_to_int(trj_obj.pose_x)
                movable_object.rectangular_box_and_accuracy.center_orientation_size.center_position_and_accuracy.metric_vehicle.y.value = self._float_to_int(trj_obj.pose_y)
                movable_object.rectangular_box_and_accuracy.center_orientation_size.center_position_and_accuracy.metric_vehicle.z.value = self._float_to_int(trj_obj.pose_z)

            # orientation of yaw, pitch, row
            movable_object.rectangular_box_and_accuracy.center_orientation_size.orientation_and_accuracy.euler_vehicle.roll.value = self._float_to_int(trj_obj.orientation_x)
            movable_object.rectangular_box_and_accuracy.center_orientation_size.orientation_and_accuracy.euler_vehicle.pitch.value = self._float_to_int(trj_obj.orientation_y)
            movable_object.rectangular_box_and_accuracy.center_orientation_size.orientation_and_accuracy.euler_vehicle.yaw.value = self._float_to_int(trj_obj.orientation_z)

            # velocity in x, y, z
            movable_object.speed_and_accuracy.metric_vehicle.x.value = self._float_to_int(trj_obj.vel_x)
            movable_object.speed_and_accuracy.metric_vehicle.y.value = self._float_to_int(trj_obj.vel_y)
            movable_object.speed_and_accuracy.metric_vehicle.z.value = self._float_to_int(trj_obj.vel_z)

            # object type
            #print(trj_obj.object_class)
            if trj_obj.object_class in self.object_type_matching:
                movable_object.type_and_confidence.type = self.trans_object_type(trj_obj.object_class)
            else:
                movable_object.type_and_confidence.type = self.trans_object_type('')

            # classification confidence
            movable_object.type_and_confidence.confidence.value = self._float_to_int(trj_obj.classification_confidence)

            # detection confidence
            movable_object.existence_confidence.value = self._float_to_int(trj_obj.detection_confidence)

            # covariance, which has not be specified yet
            for ros_ele in trj_obj.covariance:
                sensoris_ele = Int64Value()
                sensoris_ele.value = self._float_to_int(ros_ele)
                movable_object.covariance_matrix.append(sensoris_ele)

            event_group.object_detection_category.movable_object.append(movable_object)
        print("Number of tracks = " + str(len(event_group.object_detection_category.movable_object)))
        data_message.event_group.append(event_group)
        sensoris_msg = DataMessages()
        sensoris_msg.data_message.append(data_message)
        #print(sensoris_msg)
        sensoris_bytes = sensoris_msg.SerializeToString()

        return sensoris_bytes

    def decode(self, sensoris_bytes):
        ros_msg = TrackObjectList()
        sensoris_msg = DataMessages()
        sensoris_msg.ParseFromString(sensoris_bytes)

        ret_code = self._check_sensoris_msg(self.print_logs, sensoris_msg)
        if ret_code > 0:

            for data_message in sensoris_msg.data_message:

                if len(data_message.event_group) == 0:
                    if self.print_logs:
                        print("SensorisParser: Received Sensoris message with 0 sensoris event_group")
                    ret_code = 0
                    return ret_code, ros_msg

                for event_group in data_message.event_group:

                    # initialize an "objectlist" based on ros data structure
                    ros_msg = TrackObjectList()

                    # sensor source
                    if data_message.event_source[0].source.id.value in self.mast_id_matching:
                        ros_msg.mast_id = self.trans_mast_id(data_message.event_source[0].source.id.value)
                    else:
                        ros_msg.mast_id = self.mast_id_matching[-1]

                    if data_message.event_source[1].source.id.value in self.sensor_id_matching:
                        ros_msg.sensor_id = self.trans_sensor_id(data_message.event_source[1].source.id.value)
                    else:
                        ros_msg.sensor_id = self.sensor_id_matching[-1]

                    ros_msg.station_id = str(data_message.event_source[2].source.id.value)

                    # frame_id out of header or seq in header, frame_id in header is not used.
                    ros_msg.frame_id = data_message.envelope.ids.message_id.value

                    # timestampe in header, nsecs is not used
                    stamp = event_group.envelope.origin.timestamp

                    ros_msg.header.stamp.secs = int(str(stamp.posix_time.value)[0:10])
                    ros_msg.header.stamp.nsecs = int(int(str(stamp.posix_time.value)[10:16])*1e3)

                    # enumerate the object in object list
                    for object_index, movable_object in enumerate(event_group.object_detection_category.movable_object):

                        # initialize a "TrackedObject" based on ros data structure
                        tracked_object = TrackObject()

                        # timestampe in each object, nsecs is not used
                        #tracked_object.timestamp.secs = stamp.posix_time.value
                        tracked_object.timestamp.secs = ros_msg.header.stamp.secs
                        tracked_object.timestamp.nsecs = ros_msg.header.stamp.nsecs

                        # If it is failed while parsing, then: ret_code = -1, obj_list = None
                        # try:
                        # tracking id
                        tracked_object.object_id = movable_object.object_id.value

                        tracked_object.box_length_x = self._int_to_float(
                            movable_object.rectangular_box_and_accuracy.center_orientation_size.size_and_accuracy.metric_vehicle.x.value)
                        tracked_object.box_length_y = self._int_to_float(
                            movable_object.rectangular_box_and_accuracy.center_orientation_size.size_and_accuracy.metric_vehicle.y.value)
                        tracked_object.box_length_z = self._int_to_float(
                            movable_object.rectangular_box_and_accuracy.center_orientation_size.size_and_accuracy.metric_vehicle.z.value)


                        if ((data_message.event_source[1].source.id.value == 5 and movable_object.object_id.value == 0)
                            or data_message.event_source[1].source.id.value == 2):
                            # position in longitude, latitude, attitude
                            tracked_object.longitude = self._int_to_float(movable_object.rectangular_box_and_accuracy.center_orientation_size.center_position_and_accuracy.geographic_wgs84.longitude.value, 8)
                            tracked_object.latitude = self._int_to_float(movable_object.rectangular_box_and_accuracy.center_orientation_size.center_position_and_accuracy.geographic_wgs84.latitude.value, 8)
                            tracked_object.altitude = self._int_to_float(movable_object.rectangular_box_and_accuracy.center_orientation_size.center_position_and_accuracy.geographic_wgs84.altitude.value, 8)
                        else:
                            # position in x, y, z
                            tracked_object.pose_x = self._int_to_float(movable_object.rectangular_box_and_accuracy.center_orientation_size.center_position_and_accuracy.metric_vehicle.x.value)
                            tracked_object.pose_y = self._int_to_float(movable_object.rectangular_box_and_accuracy.center_orientation_size.center_position_and_accuracy.metric_vehicle.y.value)
                            tracked_object.pose_z = self._int_to_float(movable_object.rectangular_box_and_accuracy.center_orientation_size.center_position_and_accuracy.metric_vehicle.z.value)


                        # orientation of yaw, pitch, row
                        tracked_object.orientation_z = self._int_to_float(
                            movable_object.rectangular_box_and_accuracy.center_orientation_size.orientation_and_accuracy.euler_vehicle.yaw.value)
                        tracked_object.orientation_y = self._int_to_float(
                            movable_object.rectangular_box_and_accuracy.center_orientation_size.orientation_and_accuracy.euler_vehicle.pitch.value)
                        tracked_object.orientation_x = self._int_to_float(
                            movable_object.rectangular_box_and_accuracy.center_orientation_size.orientation_and_accuracy.euler_vehicle.roll.value)

                        # velocity in x, y, z
                        tracked_object.vel_x = self._int_to_float(
                            movable_object.speed_and_accuracy.metric_vehicle.x.value)
                        tracked_object.vel_y = self._int_to_float(
                            movable_object.speed_and_accuracy.metric_vehicle.y.value)
                        tracked_object.vel_z = self._int_to_float(
                            movable_object.speed_and_accuracy.metric_vehicle.z.value)

                        # object type
                        if movable_object.type_and_confidence.type in self.object_type_matching:
                            tracked_object.object_class = self.trans_object_type(
                                movable_object.type_and_confidence.type)
                        else:
                            tracked_object.object_class = self.object_type_matching[0]

                        # classification confidence
                        tracked_object.classification_confidence = self._int_to_float(
                            movable_object.type_and_confidence.confidence.value)

                        # detection confidence
                        tracked_object.detection_confidence = self._int_to_float(
                            movable_object.existence_confidence.value)

                        # covariance, which has not be specified yet
                        for sensoris_ele in movable_object.covariance_matrix:
                            ros_ele = self._int_to_float(sensoris_ele.value)
                            tracked_object.covariance.append(ros_ele)

                        # except:
                            # print("SensorisParser: Failed while parsing sensoris")
                            # print(movable_object)
                            # return -2, None

                        ros_msg.object_list.append(tracked_object)

                    # number of object
                    ros_msg.total_objects = len(ros_msg.object_list)

            # Check the converted object list
            if not ros_msg:
                if self.print_logs:
                    print("SensorisParser: Object list has not been found in message")
                ret_code = -1

            elif ros_msg.total_objects == 0:
                if self.print_logs:
                    print("SensorisParser: Object has not been found in object list")
                ret_code = -1

        return ret_code, ros_msg


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

