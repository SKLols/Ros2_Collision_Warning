/**
 * @file warning_generator.h
 * @author Michael Wittmann (miw2006@thi.de)
 * @brief Header for Warning Generator class.
 * @version 0.1
 * @date 2022-04-24
 * 
 * @copyright Copyright (c) 2022
 * @author Sourabh Lolge (sol9471@thi.de)
 * @date 2024-12-02
 * @brief Updated from ROS 1 to ROS 2.
 * 
 */

#include "ros2_collision_detection/warning_generator.hpp"

WarningGenerator::WarningGenerator(rclcpp::Publisher<v2xvf_interfaces::msg::CollisionCheckResult>::SharedPtr publisher)
:warning_generator_algorithm(nullptr),
collision_warning_publisher(publisher)
{
    RCLCPP_DEBUG(node_handle->get_logger(),"WarningGenerator::WarningGenerator constructor.");
}

void WarningGenerator::setWarningGeneratorAlgorithm(std::shared_ptr<WarningGeneratorAlgorithm> algorithm)
{
    warning_generator_algorithm.swap(algorithm);
    RCLCPP_DEBUG(node_handle->get_logger(),"WarningGenerator::setWarningGeneratorAlgorithm: set new algorithm.");
}

void WarningGenerator::setCollisionWarningPublisher(rclcpp::Publisher<v2xvf_interfaces::msg::CollisionCheckResult>::SharedPtr publisher)
{
    collision_warning_publisher = publisher;
    RCLCPP_DEBUG(node_handle->get_logger(),"TTCCalculator::setCollisionWarningPublisher: set new publisher.");
}

void WarningGenerator::createWarning(
        const v2xvf_interfaces::msg::SubjectVehicleMotion::ConstSharedPtr& subject_vehicle_motion_msg, 
        const v2xvf_interfaces::msg::PerceivedObjectMotion::ConstSharedPtr& perceived_object_motion_msg,
        double ttc
    )
{
    v2xvf_interfaces::msg::CollisionCheckResult collision_check_msg; //!< the message to be published
    collision_check_msg.header.stamp = rclcpp::Clock().now();
    collision_check_msg.perceived_object = *perceived_object_motion_msg;
    collision_check_msg.ttc = (float) ttc;  // convert double to float for publishingcollision_check_msg.result_type = warning_generator_algorithm->generateWarning(subject_vehicle_motion_msg, perceived_object_motion_msg, ttc);

    RCLCPP_INFO(node_handle->get_logger(),"WarningGenerator: perceived object with ID %d has TTC %f with result type %d.", perceived_object_motion_msg->object_movement.id, collision_check_msg.ttc, collision_check_msg.result_type);
    collision_warning_publisher->publish(collision_check_msg);
}


