cmake_minimum_required(VERSION 3.5)
project(ros2_collision_detection_plugins)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(ros2_collision_detection REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(GSL REQUIRED) #newly added
find_package(rclpy REQUIRED) #newly added
find_package(v2xvf_interfaces REQUIRED) #NEWLY ADDED
find_package(Boost REQUIRED) #Newly Added

#Add gsl Library
find_package( PkgConfig REQUIRED)


#add_library(circle_algorithm src/circle_algorithm.cpp)
#add_library(circle_equation_solver src/circle_equation_solver.cpp)
#add_library(n_circle_algorithm src/n_circle_algorithm.cpp)
add_library(ttc_algorithm_plugins src/ttc_algorithm_plugins.cpp)
add_library(warning_generator_algorithm_plugins src/warning_generator_algorithm_plugins.cpp)


#target_include_directories(circle_algorithm PUBLIC
#  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#  $<INSTALL_INTERFACE:include>)

#target_include_directories(circle_equation_solver PUBLIC
#  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#  $<INSTALL_INTERFACE:include>)

#target_include_directories(n_circle_algorithm PUBLIC
#  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#  $<INSTALL_INTERFACE:include>)

target_include_directories(ttc_algorithm_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${gsl_LIBRARIES})

  target_include_directories(warning_generator_algorithm_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

#ament_target_dependencies(
#  circle_algorithm
#  "ros2_collision_detection"
#  "pluginlib"
#  "rclcpp"
#  "rclpy"
#  "v2xvf_interfaces"
#  "Boost"
#)

#ament_target_dependencies(
#  circle_equation_solver
#  "ros2_collision_detection"
#  "pluginlib"
#  "rclcpp"
#  "rclpy"
#  "v2xvf_interfaces"
#  "Boost"
#)

#ament_target_dependencies(
#  n_circle_algorithm
#  "ros2_collision_detection"
#  "pluginlib"
#  "rclcpp"
#  "rclpy"
#  "v2xvf_interfaces"
#  "Boost"
#)

ament_target_dependencies(
  ttc_algorithm_plugins
  "ros2_collision_detection"
  "pluginlib"
  "rclcpp"
  "rclpy"
  "v2xvf_interfaces"
  "Boost"
)

ament_target_dependencies(
  warning_generator_algorithm_plugins
  "ros2_collision_detection"
  "pluginlib"
  "rclcpp"
  "rclpy"
  "v2xvf_interfaces"
  "Boost"
)

pluginlib_export_plugin_description_file(ros2_collision_detection ttc_algorithm_plugins.xml warning_generator_algorithm_plugins.xml)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(ttc_algorithm_plugins PRIVATE "ROS2_COLLISION_DETECTION_PLUGINS_BUILDING_LIBRARY")
target_compile_definitions(warning_generator_algorithm_plugins PRIVATE "ROS2_COLLISION_DETECTION_PLUGINS_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS 
  #circle_algorithm
  #circle_equation_solver
  #n_circle_algorithm
  ttc_algorithm_plugins
  warning_generator_algorithm_plugins

  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  #circle_algorithm
  #circle_equation_solver
  #n_circle_algorithm
  ttc_algorithm_plugins
  warning_generator_algorithm_plugins
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
