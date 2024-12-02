cmake_minimum_required(VERSION 3.5)
project(ros2_collision_detection)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(v2xvf_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Boost REQUIRED)  # Added Boost dependency

#Add gsl Library
find_package( PkgConfig REQUIRED)

pkg_check_modules( gsl REQUIRED gsl )

add_executable(ros2_collision_detection__node src/ros2_collision_detection__node.cpp src/warning_generator.cpp src/ttc_only_warning_algorithm.cpp)
target_include_directories(ros2_collision_detection__node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  ros2_collision_detection__node
  "pluginlib"
  "v2xvf_interfaces"
  "rclcpp"
  "Boost"  # Added Boost as a dependency
)

target_link_libraries( ros2_collision_detection__node ${gsl_LIBRARIES} )

# Link Boost libraries
target_link_libraries(ros2_collision_detection__node
  ${Boost_LIBRARIES}  # Link Boost shared_ptr library
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS ros2_collision_detection__node
  DESTINATION lib/${PROJECT_NAME})

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

ament_package()
