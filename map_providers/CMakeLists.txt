cmake_minimum_required(VERSION 3.8)
project(map_providers)

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
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)


add_executable(map_providers src/cpm_provider.cpp  src/launcher.cpp)
ament_target_dependencies(map_providers rclcpp std_msgs sensor_msgs v2xvf_interfaces ros_etsi_its_msgs geometry_msgs)

install(TARGETS 
  map_providers
  DESTINATION lib/${PROJECT_NAME}
)
# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)

  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
