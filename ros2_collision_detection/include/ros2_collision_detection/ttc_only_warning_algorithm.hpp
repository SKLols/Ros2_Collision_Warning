/**
 * @file ttc_only_warning_algorithm.h
 * @author Michael Wittmann (miw2006@thi.de)
 * @brief Header for the TTC Only Warning Algorithm class.
 * @version 0.1
 * @date 2022-04-30
 * 
 * @copyright Copyright (c) 2022
 * @author Sourabh Lolge (sol9471@thi.de)
 * @date 2024-12-02
 * @brief Updated from ROS 1 to ROS 2.
 * 
 */

#ifndef _TTC_ONLY_WARNING_ALGORITHM_HPP_
#define _TTC_ONLY_WARNING_ALGORITHM_HPP_


#include <rclcpp/rclcpp.hpp>

#include "ros2_collision_detection/warning_generator_algorithm.hpp"


/**
 * @brief Class that generates collision warnings based on TTC thresholds only.
 * 
 */
class TTCOnlyWarningAlgorithm: public WarningGeneratorAlgorithm
{
public:
    std::shared_ptr<rclcpp::Node> node_handle;

    /**
     * @brief Construct a new TTCOnlyWarningAlgorithm object.
     * 
     */
    TTCOnlyWarningAlgorithm();

    /**
     * @brief Generate a ResultType warning from the Subject Vehicle Motion, the Perceived Object Motion and the TTC only based on TTC thresholds.
     * 
     * @param subject_vehicle_motion_msg The Subject Vehicle Motion message used for TTC calculation.
     * @param perceived_object_motion_msg The Perceived Object Motion message used for TTC calculation.
     * @param ttc The Time-To-Collision computed between the Subject Vehicle and the Perceived Object.
     * @return Warning Result Type that defines the collision warning level based on TTC thresholds.
     */
    ResultType generateWarning(
            const v2xvf_interfaces::msg::SubjectVehicleMotion::SharedPtr& subject_vehicle_motion_msg, 
            const v2xvf_interfaces::msg::PerceivedObjectMotion::SharedPtr& perceived_object_motion_msg,
            double ttc
     ) override;
};

#endif  // _TTC_ONLY_WARNING_ALGORITHM_HPP_