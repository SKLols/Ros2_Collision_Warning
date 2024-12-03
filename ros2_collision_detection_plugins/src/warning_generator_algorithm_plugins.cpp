
#include "ros2_collision_detection/warning_generator_algorithm.hpp"
#include "ros2_collision_detection/ttc_only_warning_algorithm.hpp"

#include <pluginlib/class_list_macros.hpp>

//PLUGINLIB_EXPORT_CLASS(ros2_collision_detection::TTCOnlyWarningAlgorithm, ros2_collision_detection::WarningGeneratorAlgorithm)
PLUGINLIB_EXPORT_CLASS(TTCOnlyWarningAlgorithm, WarningGeneratorAlgorithm)


