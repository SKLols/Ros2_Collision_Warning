# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: sensoris/protobuf/categories/brake.proto
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import descriptor_pool as _descriptor_pool
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from sensoris.protobuf.types import base_pb2 as sensoris_dot_protobuf_dot_types_dot_base__pb2


DESCRIPTOR = _descriptor_pool.Default().AddSerializedFile(b'\n(sensoris/protobuf/categories/brake.proto\x12\"sensoris.protobuf.categories.brake\x1a\"sensoris/protobuf/types/base.proto\"\x93\x03\n\x12\x42rakeSystemsStatus\x12=\n\x08\x65nvelope\x18\x01 \x01(\x0b\x32+.sensoris.protobuf.types.base.EventEnvelope\x12>\n\nabs_status\x18\x02 \x01(\x0e\x32*.sensoris.protobuf.types.base.SystemStatus\x12>\n\nesc_status\x18\x03 \x01(\x0e\x32*.sensoris.protobuf.types.base.SystemStatus\x12>\n\ntcs_status\x18\x04 \x01(\x0e\x32*.sensoris.protobuf.types.base.SystemStatus\x12>\n\nebd_status\x18\x05 \x01(\x0e\x32*.sensoris.protobuf.types.base.SystemStatus\x12>\n\neba_status\x18\x06 \x01(\x0e\x32*.sensoris.protobuf.types.base.SystemStatus\"\xa7\x01\n\rBrakeCategory\x12@\n\x08\x65nvelope\x18\x01 \x01(\x0b\x32..sensoris.protobuf.types.base.CategoryEnvelope\x12T\n\x14\x62rake_systems_status\x18\x02 \x03(\x0b\x32\x36.sensoris.protobuf.categories.brake.BrakeSystemsStatusBh\n\x1dorg.sensoris.categories.brakeB\x15SensorisBrakeCategoryP\x01Z+sensoris.org/specification/categories/brake\xf8\x01\x01\x62\x06proto3')



_BRAKESYSTEMSSTATUS = DESCRIPTOR.message_types_by_name['BrakeSystemsStatus']
_BRAKECATEGORY = DESCRIPTOR.message_types_by_name['BrakeCategory']
BrakeSystemsStatus = _reflection.GeneratedProtocolMessageType('BrakeSystemsStatus', (_message.Message,), {
  'DESCRIPTOR' : _BRAKESYSTEMSSTATUS,
  '__module__' : 'sensoris.protobuf.categories.brake_pb2'
  # @@protoc_insertion_point(class_scope:sensoris.protobuf.categories.brake.BrakeSystemsStatus)
  })
_sym_db.RegisterMessage(BrakeSystemsStatus)

BrakeCategory = _reflection.GeneratedProtocolMessageType('BrakeCategory', (_message.Message,), {
  'DESCRIPTOR' : _BRAKECATEGORY,
  '__module__' : 'sensoris.protobuf.categories.brake_pb2'
  # @@protoc_insertion_point(class_scope:sensoris.protobuf.categories.brake.BrakeCategory)
  })
_sym_db.RegisterMessage(BrakeCategory)

if _descriptor._USE_C_DESCRIPTORS == False:

  DESCRIPTOR._options = None
  DESCRIPTOR._serialized_options = b'\n\035org.sensoris.categories.brakeB\025SensorisBrakeCategoryP\001Z+sensoris.org/specification/categories/brake\370\001\001'
  _BRAKESYSTEMSSTATUS._serialized_start=117
  _BRAKESYSTEMSSTATUS._serialized_end=520
  _BRAKECATEGORY._serialized_start=523
  _BRAKECATEGORY._serialized_end=690
# @@protoc_insertion_point(module_scope)