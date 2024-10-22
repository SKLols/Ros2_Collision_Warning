cmake_minimum_required(VERSION 3.8)
project(map_manager)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(v2xvf_interfaces REQUIRED)
find_package(ros_etsi_its_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosbridge_server REQUIRED)
find_package(rclpy REQUIRED)

include_directories(include)

add_executable(map_manager_node_ros2 src/map_manager.cpp)
ament_target_dependencies(map_manager_node_ros2 rclcpp std_msgs sensor_msgs v2xvf_interfaces ros_etsi_its_msgs rosbridge_server rclpy)

install(TARGETS 
  map_manager_node_ros2
  DESTINATION lib/${PROJECT_NAME}
)
# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
ament_package()
