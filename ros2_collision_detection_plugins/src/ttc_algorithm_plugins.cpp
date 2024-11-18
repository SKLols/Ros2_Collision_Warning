#include "ros2_collision_detection/ttc_algorithm.hpp"
#include "ros2_collision_detection/circle_algorithm.hpp"
#include "ros2_collision_detection/n_circle_algorithm.hpp"



#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(ros2_collision_detection_plugins::CircleAlgorithmPlugins, ros2_collision_detection::TTCAlgorithm)


//namespace ros2_collision_detection {
//
//}
//
//#include <pluginlib/class_list_macros.hpp>
//
//
//PLUGINLIB_EXPORT_CLASS(ros2_collision_detection_plugins::CircleAlgorithm, ros2_collision_detection::TTCAlgorithm)
//PLUGINLIB_EXPORT_CLASS(ros2_collision_detection_plugins::NCircleAlgorithm, ros2_collision_detection::TTCAlgorithm)
