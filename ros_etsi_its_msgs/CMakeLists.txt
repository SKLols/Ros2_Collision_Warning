cmake_minimum_required(VERSION 3.5)
project(ros_etsi_its_msgs)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AccelerationControl.msg"
  "msg/ActionID.msg"
  "msg/Altitude.msg"
  "msg/BasicVehicleContainerHighFrequency.msg"
  "msg/BasicVehicleContainerLowFrequency.msg"
  "msg/CAM.msg"
  "msg/CauseCode.msg"
  "msg/CBR.msg"
  "msg/CPM.msg"
  "msg/Curvature.msg"
  "msg/CurvatureCalculationMode.msg"
  "msg/DeltaReferencePosition.msg"
  "msg/DENM.msg"
  "msg/DriveDirection.msg"
  "msg/DynamicStatus.msg"
  "msg/EventPoint.msg"
  "msg/ExteriorLights.msg"
  "msg/Heading.msg"
  "msg/InformationQuality.msg"
  "msg/ItsPduHeader.msg"
  "msg/LateralAcceleration.msg"
  "msg/ListOfPerceivedObjects.msg"
  "msg/LocationContainer.msg"
  "msg/LongitudinalAcceleration.msg"
  "msg/ManagementContainer.msg"
  "msg/MatchedPosition.msg"
  "msg/ObjectClassDescription.msg"
  "msg/ObjectDimension.msg"
  "msg/ObjectDistanceWithConfidence.msg"
  "msg/ObjectRefPoint.msg"
  "msg/OriginatingVehicleContainer.msg"
  "msg/PathDeltaTime.msg"
  "msg/PathHistory.msg"
  "msg/PathPoint.msg"
  "msg/PerceivedObject.msg"
  "msg/PositionConfidenceEllipse.msg"
  "msg/ReferencePosition.msg"
  "msg/RelevanceDistance.msg"
  "msg/RelevanceTrafficDirection.msg"
  "msg/SensorInformation.msg"
  "msg/SensorInformationContainer.msg"
  "msg/SituationContainer.msg"
  "msg/Speed.msg"
  "msg/SpeedExtended.msg"
  "msg/StationType.msg"
  "msg/VehicleLength.msg"
  "msg/VehicleRole.msg"
  "msg/VehicleSensor.msg"
  "msg/VehicleSensorProperty.msg"
  "msg/VehicleWidth.msg"
  "msg/WGS84Angle.msg"
  "msg/YawRate.msg"
  "msg/GenericLane.msg"
  "msg/IntersectionGeometry.msg"
  "msg/Intersections.msg"
  "msg/Connection.msg"
  "msg/NodeSetXY.msg"
  "msg/SPATEM.msg"
  "msg/IntersectionState.msg"
  "msg/MovementState.msg"
  "msg/MovementEvent.msg"
  "msg/MAPEM.msg"
  DEPENDENCIES std_msgs
)

#generate_messages(DEPENDENCIES std_msgs)

#ament_package(
  #INCLUDE_DIRS include
#  LIBRARY_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/lib
#  #RUNTIME_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/bin
#  INCLUDE_FILES manifest.xml
#  DEPENDS rclcpp std_msgs
#)

ament_package()

install(DIRECTORY include/
  DESTINATION include/
  FILES_MATCHING PATTERN "*.h"
)

