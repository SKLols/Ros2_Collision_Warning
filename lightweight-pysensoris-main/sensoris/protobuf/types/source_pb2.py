# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: sensoris/protobuf/types/source.proto
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import descriptor_pool as _descriptor_pool
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from google.protobuf import any_pb2 as google_dot_protobuf_dot_any__pb2
from google.protobuf import wrappers_pb2 as google_dot_protobuf_dot_wrappers__pb2
from sensoris.protobuf.types import base_pb2 as sensoris_dot_protobuf_dot_types_dot_base__pb2
from sensoris.protobuf.types import spatial_pb2 as sensoris_dot_protobuf_dot_types_dot_spatial__pb2


DESCRIPTOR = _descriptor_pool.Default().AddSerializedFile(b'\n$sensoris/protobuf/types/source.proto\x12\x1esensoris.protobuf.types.source\x1a\x19google/protobuf/any.proto\x1a\x1egoogle/protobuf/wrappers.proto\x1a\"sensoris/protobuf/types/base.proto\x1a%sensoris/protobuf/types/spatial.proto\"\xe0\x08\n\x19NavigationSatelliteSystem\x12\x63\n\x10satellite_system\x18\x01 \x03(\x0e\x32I.sensoris.protobuf.types.source.NavigationSatelliteSystem.SatelliteSystem\x12\x87\x01\n#satellite_based_augmentation_system\x18\x02 \x03(\x0e\x32Z.sensoris.protobuf.types.source.NavigationSatelliteSystem.SatelliteBasedAugmentationSystem\x12\x81\x01\n ground_based_augmentation_system\x18\x03 \x03(\x0e\x32W.sensoris.protobuf.types.source.NavigationSatelliteSystem.GroundBasedAugmentationSystem\x12\x39\n\x0e\x65levation_mask\x18\x04 \x01(\x0b\x32\x1b.google.protobuf.Int64ValueB\x04\x88\xb5\x18\x00\x12`\n\x1b\x61ntenna_offset_and_accuracy\x18\x05 \x01(\x0b\x32\x35.sensoris.protobuf.types.spatial.XyzVectorAndAccuracyB\x04\x88\xb5\x18\x00\x12\'\n\textension\x18\x0f \x03(\x0b\x32\x14.google.protobuf.Any\"\x83\x01\n\x0fSatelliteSystem\x12\x1c\n\x18UNKNOWN_SATELLITE_SYSTEM\x10\x00\x12\x07\n\x03GPS\x10\x01\x12\x0b\n\x07GLONASS\x10\x02\x12\x0b\n\x07GALILEO\x10\x03\x12\x0c\n\x08\x42\x45IDOU_1\x10\x04\x12\x0c\n\x08\x42\x45IDOU_2\x10\x05\x12\t\n\x05NAVIC\x10\x06\x12\x08\n\x04QZSS\x10\x07\"\xe2\x01\n SatelliteBasedAugmentationSystem\x12\x1b\n\x17UNKNOWN_SATELLITE_BASED\x10\x00\x12\x13\n\x0fSATELLITE_BASED\x10\x01\x12\x08\n\x04WAAS\x10\x02\x12\t\n\x05\x45GNOS\x10\x03\x12\x08\n\x04MSAS\x10\x04\x12\x18\n\x14QZSS_SATELLITE_BASED\x10\x05\x12\t\n\x05GAGAN\x10\x06\x12\x08\n\x04SDCM\x10\x07\x12\x08\n\x04SNAS\x10\x08\x12\x08\n\x04WAGE\x10\t\x12\r\n\tSTAR_FIRE\x10\n\x12\x0c\n\x08STAR_FIX\x10\x0b\x12\r\n\tOMNI_STAR\x10\x0c\"\x9e\x01\n\x1dGroundBasedAugmentationSystem\x12\x18\n\x14UNKNOWN_GROUND_BASED\x10\x00\x12\x10\n\x0cGROUND_BASED\x10\x01\x12\x08\n\x04GBAS\x10\x02\x12\t\n\x05NDGPS\x10\x03\x12\t\n\x05SAPOS\x10\x04\x12\x07\n\x03\x41LF\x10\x05\x12\x0c\n\x08\x41XIO_NET\x10\x06\x12\x0b\n\x07VRS_NOW\x10\x07\x12\r\n\tSMART_NET\x10\x08\"\xbb\x03\n\x06Sensor\x12p\n!mounting_position_and_orientation\x18\x01 \x01(\x0b\x32\x45.sensoris.protobuf.types.source.Sensor.MountingPositionAndOrientation\x12`\n\x1bnavigation_satellite_system\x18\x02 \x01(\x0b\x32\x39.sensoris.protobuf.types.source.NavigationSatelliteSystemH\x00\x1a\xd0\x01\n\x1eMountingPositionAndOrientation\x12V\n\x18translation_and_accuracy\x18\x01 \x01(\x0b\x32\x34.sensoris.protobuf.types.spatial.PositionAndAccuracy\x12V\n\x18orientation_and_accuracy\x18\x02 \x01(\x0b\x32\x34.sensoris.protobuf.types.spatial.RotationAndAccuracyB\n\n\x08specific\">\n\x0cSensorFusion\x12.\n\tsensor_id\x18\x01 \x03(\x0b\x32\x1b.google.protobuf.Int64Value\"\xe7\x02\n\x06Source\x12\'\n\x02id\x18\x01 \x01(\x0b\x32\x1b.google.protobuf.Int64Value\x12\x34\n\x06\x65ntity\x18\x02 \x01(\x0b\x32$.sensoris.protobuf.types.base.Entity\x12J\n\x12sampling_frequency\x18\x03 \x01(\x0b\x32(.sensoris.protobuf.types.base.Int64ValueB\x04\x88\xb5\x18\x03\x12\x38\n\x06sensor\x18\x04 \x01(\x0b\x32&.sensoris.protobuf.types.source.SensorH\x00\x12\x45\n\rsensor_fusion\x18\x05 \x01(\x0b\x32,.sensoris.protobuf.types.source.SensorFusionH\x00\x12\'\n\textension\x18\x0f \x03(\x0b\x32\x14.google.protobuf.AnyB\x08\n\x06sourceB^\n\x19org.sensoris.types.sourceB\x13SensorisSourceTypesP\x01Z\'sensoris.org/specification/types/source\xf8\x01\x01\x62\x06proto3')



_NAVIGATIONSATELLITESYSTEM = DESCRIPTOR.message_types_by_name['NavigationSatelliteSystem']
_SENSOR = DESCRIPTOR.message_types_by_name['Sensor']
_SENSOR_MOUNTINGPOSITIONANDORIENTATION = _SENSOR.nested_types_by_name['MountingPositionAndOrientation']
_SENSORFUSION = DESCRIPTOR.message_types_by_name['SensorFusion']
_SOURCE = DESCRIPTOR.message_types_by_name['Source']
_NAVIGATIONSATELLITESYSTEM_SATELLITESYSTEM = _NAVIGATIONSATELLITESYSTEM.enum_types_by_name['SatelliteSystem']
_NAVIGATIONSATELLITESYSTEM_SATELLITEBASEDAUGMENTATIONSYSTEM = _NAVIGATIONSATELLITESYSTEM.enum_types_by_name['SatelliteBasedAugmentationSystem']
_NAVIGATIONSATELLITESYSTEM_GROUNDBASEDAUGMENTATIONSYSTEM = _NAVIGATIONSATELLITESYSTEM.enum_types_by_name['GroundBasedAugmentationSystem']
NavigationSatelliteSystem = _reflection.GeneratedProtocolMessageType('NavigationSatelliteSystem', (_message.Message,), {
  'DESCRIPTOR' : _NAVIGATIONSATELLITESYSTEM,
  '__module__' : 'sensoris.protobuf.types.source_pb2'
  # @@protoc_insertion_point(class_scope:sensoris.protobuf.types.source.NavigationSatelliteSystem)
  })
_sym_db.RegisterMessage(NavigationSatelliteSystem)

Sensor = _reflection.GeneratedProtocolMessageType('Sensor', (_message.Message,), {

  'MountingPositionAndOrientation' : _reflection.GeneratedProtocolMessageType('MountingPositionAndOrientation', (_message.Message,), {
    'DESCRIPTOR' : _SENSOR_MOUNTINGPOSITIONANDORIENTATION,
    '__module__' : 'sensoris.protobuf.types.source_pb2'
    # @@protoc_insertion_point(class_scope:sensoris.protobuf.types.source.Sensor.MountingPositionAndOrientation)
    })
  ,
  'DESCRIPTOR' : _SENSOR,
  '__module__' : 'sensoris.protobuf.types.source_pb2'
  # @@protoc_insertion_point(class_scope:sensoris.protobuf.types.source.Sensor)
  })
_sym_db.RegisterMessage(Sensor)
_sym_db.RegisterMessage(Sensor.MountingPositionAndOrientation)

SensorFusion = _reflection.GeneratedProtocolMessageType('SensorFusion', (_message.Message,), {
  'DESCRIPTOR' : _SENSORFUSION,
  '__module__' : 'sensoris.protobuf.types.source_pb2'
  # @@protoc_insertion_point(class_scope:sensoris.protobuf.types.source.SensorFusion)
  })
_sym_db.RegisterMessage(SensorFusion)

Source = _reflection.GeneratedProtocolMessageType('Source', (_message.Message,), {
  'DESCRIPTOR' : _SOURCE,
  '__module__' : 'sensoris.protobuf.types.source_pb2'
  # @@protoc_insertion_point(class_scope:sensoris.protobuf.types.source.Source)
  })
_sym_db.RegisterMessage(Source)

if _descriptor._USE_C_DESCRIPTORS == False:

  DESCRIPTOR._options = None
  DESCRIPTOR._serialized_options = b'\n\031org.sensoris.types.sourceB\023SensorisSourceTypesP\001Z\'sensoris.org/specification/types/source\370\001\001'
  _NAVIGATIONSATELLITESYSTEM.fields_by_name['elevation_mask']._options = None
  _NAVIGATIONSATELLITESYSTEM.fields_by_name['elevation_mask']._serialized_options = b'\210\265\030\000'
  _NAVIGATIONSATELLITESYSTEM.fields_by_name['antenna_offset_and_accuracy']._options = None
  _NAVIGATIONSATELLITESYSTEM.fields_by_name['antenna_offset_and_accuracy']._serialized_options = b'\210\265\030\000'
  _SOURCE.fields_by_name['sampling_frequency']._options = None
  _SOURCE.fields_by_name['sampling_frequency']._serialized_options = b'\210\265\030\003'
  _NAVIGATIONSATELLITESYSTEM._serialized_start=207
  _NAVIGATIONSATELLITESYSTEM._serialized_end=1327
  _NAVIGATIONSATELLITESYSTEM_SATELLITESYSTEM._serialized_start=806
  _NAVIGATIONSATELLITESYSTEM_SATELLITESYSTEM._serialized_end=937
  _NAVIGATIONSATELLITESYSTEM_SATELLITEBASEDAUGMENTATIONSYSTEM._serialized_start=940
  _NAVIGATIONSATELLITESYSTEM_SATELLITEBASEDAUGMENTATIONSYSTEM._serialized_end=1166
  _NAVIGATIONSATELLITESYSTEM_GROUNDBASEDAUGMENTATIONSYSTEM._serialized_start=1169
  _NAVIGATIONSATELLITESYSTEM_GROUNDBASEDAUGMENTATIONSYSTEM._serialized_end=1327
  _SENSOR._serialized_start=1330
  _SENSOR._serialized_end=1773
  _SENSOR_MOUNTINGPOSITIONANDORIENTATION._serialized_start=1553
  _SENSOR_MOUNTINGPOSITIONANDORIENTATION._serialized_end=1761
  _SENSORFUSION._serialized_start=1775
  _SENSORFUSION._serialized_end=1837
  _SOURCE._serialized_start=1840
  _SOURCE._serialized_end=2199
# @@protoc_insertion_point(module_scope)