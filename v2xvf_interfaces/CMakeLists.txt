cmake_minimum_required(VERSION 3.8)
project(v2xvf_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ros_etsi_its_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Header.msg"
  "msg/ListMapObjects.msg"
  "msg/MapObject.msg"
  "msg/AccelerationControl.msg"
  
  DEPENDENCIES ros_etsi_its_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
