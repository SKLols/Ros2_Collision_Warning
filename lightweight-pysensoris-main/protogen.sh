#!/bin/sh

# My first script

echo "Start generating the SENSORIS pd2-files"

TARGET=./proto_python/
SOURCE=sensoris/protobuf/

echo "Start generating tprotobuf/types/base.pd2"
protoc --python_out=$TARGET $SOURCE/types/base.proto 
echo "Start generating tprotobuf/types/job.pd2"
protoc --python_out=$TARGET $SOURCE/types/job.proto
echo "Start generating tprotobuf/types/collection.pd2"
protoc --python_out=$TARGET $SOURCE/types/collection.proto
echo "Start generating tprotobuf/types/job_validity.pd2"
protoc --python_out=$TARGET $SOURCE/types/job_validity.proto
echo "Start generating tprotobuf/types/logical_expression.pd2"
protoc --python_out=$TARGET $SOURCE/types/logical_expression.proto
echo "Start generating tprotobuf/types/map.pd2"
protoc --python_out=$TARGET $SOURCE/types/map.proto
echo "Start generating tprotobuf/types/source.pd2"
protoc --python_out=$TARGET $SOURCE/types/source.proto
echo "Start generating tprotobuf/types/spatial.pd2"
protoc --python_out=$TARGET $SOURCE/types/spatial.proto


protoc --python_out=$TARGET $SOURCE/categories/brake.proto
protoc --python_out=$TARGET $SOURCE/categories/driving_behavior.proto
protoc --python_out=$TARGET $SOURCE/categories/intersection_attribution.proto
protoc --python_out=$TARGET $SOURCE/categories/localization.proto
protoc --python_out=$TARGET $SOURCE/categories/map.proto
protoc --python_out=$TARGET $SOURCE/categories/object_detection.proto
protoc --python_out=$TARGET $SOURCE/categories/powertrain.proto
protoc --python_out=$TARGET $SOURCE/categories/road_attribution.proto
protoc --python_out=$TARGET $SOURCE/categories/traffic_events.proto
protoc --python_out=$TARGET $SOURCE/categories/traffic_maneuver.proto
protoc --python_out=$TARGET $SOURCE/categories/traffic_regulation.proto
protoc --python_out=$TARGET $SOURCE/categories/weather.proto
protoc --python_out=$TARGET $SOURCE/categories/vehicle_device.proto
protoc --python_out=$TARGET $SOURCE/categories/etsi_denm.proto

protoc --python_out=$TARGET $SOURCE/messages/data.proto
protoc --python_out=$TARGET $SOURCE/messages/job.proto
