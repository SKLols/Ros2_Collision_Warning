/**
 * @file n_circle_algorithm.cpp
 * @author Michael Wittmann (miw2006@thi.de)
 * @brief Implementation of the methods of extended n Circle Algorithm class.
 * @version 0.1
 * @date 2022-04-22
 * 
 * @copyright Copyright (c) 2022
 * 
 * @author Sourabh Lolge (sol9471@thi.de)
 * @date 2024-11-12
 * @brief Updated from ROS 1 to ROS 2.
 * 
 */
/*
#include <ros2_collision_detection/n_circle_algorithm.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <sstream>
#include <vector>
#include <array>

#define DEFAULT_CIRCLE_COUNT 1      //!< default number of circles if no circle_count is passed
#define POLYNOMIAL_ARRAY_LENGTH 5   //!< quartic equation has variable of degree 0 to 4

namespace ros2_collision_detection_plugins {

NCircleAlgorithm::NCircleAlgorithm(rclcpp::Node::SharedPtr node) : node_(node)
{
    RCLCPP_INFO(node_->get_logger(), "NCircleAlgorithm::NCircleAlgorithm constructed.");
}

void NCircleAlgorithm::init(parameter_map_t &parameter_map)
{
    try
    {
        // Check if "ttc_algorithm_circle_count" exists in the parameter map
        if (parameter_map.find("ttc_algorithm_circle_count") != parameter_map.end())
        {
            int circle_count = std::get<int>(parameter_map["ttc_algorithm_circle_count"]);
            if (circle_count >= 1)
            {
                n = circle_count;
                RCLCPP_INFO(node_->get_logger(), "NCircleAlgorithm::init with n = %d.", circle_count);
            }
        }
        else
        {
            n = DEFAULT_CIRCLE_COUNT;
            RCLCPP_ERROR(node_->get_logger(), "NCircleAlgorithm::init: 'ttc_algorithm_circle_count' could not be retrieved. Using default n=%d.", DEFAULT_CIRCLE_COUNT);
        }
    }
    catch (const std::out_of_range &e)
    {
        n = DEFAULT_CIRCLE_COUNT;
        RCLCPP_ERROR(node_->get_logger(), "NCircleAlgorithm::init: no 'ttc_algorithm_circle_count' found. Using default n=%d.", DEFAULT_CIRCLE_COUNT);
    }
}

std::string NCircleAlgorithm::convertMotionStructToString(const object_motion_t &object_motion)
{
    std::stringstream result;

    result << "center_pos_x = " << object_motion.center_pos_x << "\n";
    result << "center_pos_y = " << object_motion.center_pos_y << "\n";
    result << "length = " << object_motion.length << "\n";
    result << "width = " << object_motion.width << "\n";
    result << "heading = " << object_motion.heading << "\n";
    result << "speed = " << object_motion.speed << "\n";
    result << "acceleration = " << object_motion.acceleration << "\n";

    return result.str();
}

double NCircleAlgorithm::computeSinFromHeading(const float &heading)
{
    return std::sin(heading * M_PI / 180.0);
}

double NCircleAlgorithm::computeCosFromHeading(const float &heading)
{
    return std::cos(heading * M_PI / 180.0);
}

double NCircleAlgorithm::computeHeadingAdjustedValue(const float &value_to_adjust, const double &trigonometric_value)
{
    return trigonometric_value * value_to_adjust;
}

double NCircleAlgorithm::computeFrontBumperPos(const float &center_pos, const double &trigonometric_value, const float &length)
{
    double result = center_pos + (trigonometric_value * length / 2);
    RCLCPP_DEBUG(node_->get_logger(), "NCircleAlgorithm::computeFrontBumperPos: result: %f | from value: %f, trigonometry: %f, length: %f.", result, center_pos, trigonometric_value, length);
    return result;
}

std::vector<std::array<double, 2>> NCircleAlgorithm::computeAllCircleCenters(const double &front_bumper_pos_x, const double &front_bumper_pos_y, const double &sin_heading, const double &cos_heading, const double &length, const int &circle_count)
{
    std::vector<std::array<double, 2>> circles;

    for (int i = 0; i < circle_count; i++)
    {
        int factor = i + 1;
        double part_length = length / (circle_count + 1);
        std::array<double, 2> circle_i_pos;
        circle_i_pos[0] = computeCircleCenter(front_bumper_pos_x, factor, sin_heading, part_length);
        circle_i_pos[1] = computeCircleCenter(front_bumper_pos_y, factor, cos_heading, part_length);
        circles.push_back(circle_i_pos);
    }

    RCLCPP_DEBUG(node_->get_logger(), "NCircleAlgorithm::computeAllCircleCenters: all circles from: (%f,%f) with sin: %f, cos: %f, length: %f, circle_count: %d.", front_bumper_pos_x, front_bumper_pos_y, sin_heading, cos_heading, length, circle_count);
    for (const auto &circle : circles)
    {
        RCLCPP_DEBUG(node_->get_logger(), "circle center: (%f,%f).", circle[0], circle[1]);
    }
    return circles;
}

double NCircleAlgorithm::computeCircleCenter(const double &front_bumper_pos, const int &factor, const double &trigonometric_value, const double &part_length)
{
    return front_bumper_pos - factor * (trigonometric_value * part_length);
}

double NCircleAlgorithm::computeRadius(const float &length, const float &width, const int &circle_count)
{
    // radius = sqrt( (length / (n+1))^2 + (width / 2)^2 )
    double part_length = length / (n + 1);
    double half_width = width / 2;
    double result = std::sqrt(part_length * part_length + half_width * half_width);
    RCLCPP_DEBUG(node_->get_logger(), "NCircleAlgorithm::computeRadius: radius = %f | length: %f, width: %f, circle_count: %d", result, length, width, circle_count);
    return result;
}

std::vector<double> NCircleAlgorithm::calculatePossibleTTCs(const object_motion_t &subject_object_motion, const object_motion_t &perceived_object_motion, int circle_count)
{
    std::vector<double> possible_ttc_list;

    double sin_subject_obj_heading = computeSinFromHeading(subject_object_motion.heading);
    double cos_subject_obj_heading = computeCosFromHeading(subject_object_motion.heading);
    double sin_perceived_obj_heading = computeSinFromHeading(perceived_object_motion.heading);
    double cos_perceived_obj_heading = computeCosFromHeading(perceived_object_motion.heading);

    double accel_subject_obj_sin_adjusted = computeHeadingAdjustedValue(subject_object_motion.acceleration, sin_subject_obj_heading);
    double accel_subject_obj_cos_adjusted = computeHeadingAdjustedValue(subject_object_motion.acceleration, cos_subject_obj_heading);
    double accel_perceived_obj_sin_adjusted = computeHeadingAdjustedValue(perceived_object_motion.acceleration, sin_perceived_obj_heading);
    double accel_perceived_obj_cos_adjusted = computeHeadingAdjustedValue(perceived_object_motion.acceleration, cos_perceived_obj_heading);

    double speed_subject_obj_sin_adjusted = computeHeadingAdjustedValue(subject_object_motion.speed, sin_subject_obj_heading);
    double speed_subject_obj_cos_adjusted = computeHeadingAdjustedValue(subject_object_motion.speed, cos_subject_obj_heading);
    double speed_perceived_obj_sin_adjusted = computeHeadingAdjustedValue(perceived_object_motion.speed, sin_perceived_obj_heading);
    double speed_perceived_obj_cos_adjusted = computeHeadingAdjustedValue(perceived_object_motion.speed, cos_perceived_obj_heading);

    double accel_diff_sin_adjusted = accel_subject_obj_sin_adjusted - accel_perceived_obj_sin_adjusted;
    double accel_diff_cos_adjusted = accel_subject_obj_cos_adjusted - accel_perceived_obj_cos_adjusted;
    double speed_diff_sin_adjusted = speed_subject_obj_sin_adjusted - speed_perceived_obj_sin_adjusted;
    double speed_diff_cos_adjusted = speed_subject_obj_cos_adjusted - speed_perceived_obj_cos_adjusted;

    double accel_diff_square_sin_adjusted = accel_diff_sin_adjusted * accel_diff_sin_adjusted;
    double accel_diff_square_cos_adjusted = accel_diff_cos_adjusted * accel_diff_cos_adjusted;
    double speed_diff_square_sin_adjusted = speed_diff_sin_adjusted * speed_diff_sin_adjusted;
    double speed_diff_square_cos_adjusted = speed_diff_cos_adjusted * speed_diff_cos_adjusted;

    double front_bumper_pos_x_subject_obj = computeFrontBumperPos(subject_object_motion.center_pos_x, sin_subject_obj_heading, subject_object_motion.length);
    double front_bumper_pos_y_subject_obj = computeFrontBumperPos(subject_object_motion.center_pos_y, cos_subject_obj_heading, subject_object_motion.length);

    double front_bumper_pos_x_perceived_obj = computeFrontBumperPos(perceived_object_motion.center_pos_x, sin_perceived_obj_heading, perceived_object_motion.length);
    double front_bumper_pos_y_perceived_obj = computeFrontBumperPos(perceived_object_motion.center_pos_y, cos_perceived_obj_heading, perceived_object_motion.length);

    std::vector<std::array<double, 2>> circles_subject_obj = computeAllCircleCenters(front_bumper_pos_x_subject_obj, front_bumper_pos_y_subject_obj, sin_subject_obj_heading, cos_subject_obj_heading, subject_object_motion.length, n);
    std::vector<std::array<double, 2>> circles_perceived_obj = computeAllCircleCenters(front_bumper_pos_x_perceived_obj, front_bumper_pos_y_perceived_obj, sin_perceived_obj_heading, cos_perceived_obj_heading, perceived_object_motion.length, n);

    double radius = computeRadius(subject_object_motion.length, subject_object_motion.width, n);

    for (int i = 0; i < circle_count; i++)
    {
        double delta_x = circles_subject_obj[i][0] - circles_perceived_obj[i][0];
        double delta_y = circles_subject_obj[i][1] - circles_perceived_obj[i][1];

        double delta_square = delta_x * delta_x + delta_y * delta_y;
        double acceleration_term = 0.5 * (accel_diff_square_sin_adjusted + accel_diff_square_cos_adjusted);
        double speed_term = (speed_diff_square_sin_adjusted + speed_diff_square_cos_adjusted);
        double ttc = 0.0;

        if (delta_square != 0.0)
        {
            ttc = (-speed_term + std::sqrt(speed_term * speed_term + 2 * acceleration_term * delta_square)) / acceleration_term;
        }
        possible_ttc_list.push_back(ttc);
    }

    return possible_ttc_list;
}

}  // namespace ros2_collision_detection_plugins
*/