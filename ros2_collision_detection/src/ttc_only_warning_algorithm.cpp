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

#include "ros2_collision_detection/ttc_only_warning_algorithm.hpp"


TTCOnlyWarningAlgorithm::TTCOnlyWarningAlgorithm()
{
    RCLCPP_INFO(node_handle->get_logger(),"TTCOnlyWarningAlgorithm::TTCOnlyWarningAlgorithm constructed.");
}

ResultType TTCOnlyWarningAlgorithm::generateWarning(const v2xvf_interfaces::msg::SubjectVehicleMotion::SharedPtr subject_vehicle_motion_msg, 
            const v2xvf_interfaces::msg::PerceivedObjects::SharedPtr perceived_object_motion_msg,
            double ttc)
{
    // TTC thresholds like Honda Collision Mitgation Braking System (CMBS)
    // see https://doi.org/10.1016/j.sbspro.2011.08.075
    if(ttc > 10)
    {
        return RESULT_IGNORE;
    }
    else if(ttc <= 10 && ttc > 3)
    {
        return RESULT_CLOSE_MONITORING;
    }
    else if(ttc <= 3 && ttc > 2)
    {
        return RESULT_WARNING;
    }
    else
    {
        // ttc <= 2
        return RESULT_ALERT;
    }
}