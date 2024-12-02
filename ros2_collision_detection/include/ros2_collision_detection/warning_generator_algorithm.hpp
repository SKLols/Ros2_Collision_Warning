/**
 * @file warning_generator_algorithm.h
 * @author Michael Wittmann (miw2006@thi.de)
 * @brief Header for Warning Generator Algorithm interface.
 * @version 0.1
 * @date 2022-04-24
 * 
 * @copyright Copyright (c) 2022
 * 
 * @author Sourabh Lolge (sol9471@thi.de)
 * @date 2024-12-02
 * @brief Updated from ROS 1 to ROS 2.
 * 
 */

#ifndef _WARNING_GENERATOR_ALGORITHM_HPP_
#define _WARNING_GENERATOR_ALGORITHM_HPP_


#include <v2xvf_interfaces/msg/perceived_objects.hpp>
#include <v2xvf_interfaces/msg/subject_vehicle_motion.hpp>

#include "ros2_collision_detection/enum_result_type.hpp"


/**
 * @brief Interface that defines the warning generation method that all concrete Warning Generator Algorithm classes must implement.
 * 
 */
class WarningGeneratorAlgorithm
{
    public:
        /**
         * @brief Generate a ResultType warning from the Subject Vehicle Motion, the Perceived Object Motion and the Time-To-Collision between the two motions.
         * 
         * @param subject_vehicle_motion_msg The Subject Vehicle Motion message used for TTC calculation.
         * @param perceived_object_motion_msg The Perceived Object Motion message used for TTC calculation.
         * @param ttc The Time-To-Collision computed between the Subject Vehicle and the Perceived Object.
         * @return Warning Result Type that defines the collision warning level.
         */
        virtual ResultType generateWarning(
            const v2xvf_interfaces::msg::SubjectVehicleMotion::SharedPtr subject_vehicle_motion_msg, 
            const v2xvf_interfaces::msg::PerceivedObjects::SharedPtr perceived_object_motion_msg,
            double ttc
        ) = 0;

};

#endif // _WARNING_GENERATOR_ALGORITHM_HPP_