cmake_minimum_required(VERSION 3.5)
project(ros2_collision_detection)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
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
find_package(pluginlib REQUIRED)


# Include directories //We used target_include_directories instead of this
#include_directories(include)

# Create the library target for the plugin
set(PLUGIN_SOURCES
  src/ttc_algorithm_plugin.cpp  # Add your new plugin source file
  # You can include other plugin source files here as necessary
)

add_library(collision_detection_plugin ${PLUGIN_SOURCES})

# Create the executable for the main application
add_executable(collision_detection src/collision_detection.cpp)

# Specify include directories for both the library and the executable
target_include_directories(collision_detection_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(collision_detection PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Export the include directories for dependent packages
ament_export_include_directories(
  include
)

# Specify dependencies for both targets
ament_target_dependencies(collision_detection_plugin rclcpp v2xvf_interfaces pluginlib)
ament_target_dependencies(collision_detection rclcpp v2xvf_interfaces pluginlib)

# Plugin export
pluginlib_export_plugin_description_file(ros2_collision_detection ttc_algorithm_plugins.xml)

# Install the plugin library
install(TARGETS 
  collision_detection_plugin
  DESTINATION lib/${PROJECT_NAME})

#install so that ros2 run can find your executable
install(TARGETS 
  collision_detection
  DESTINATION lib/${PROJECT_NAME})

# Install include files
install(DIRECTORY include/
  DESTINATION include
)

#Install plugin description file
install(FILES ttc_algorithm_plugins.xml
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
