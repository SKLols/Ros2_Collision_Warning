/**
 * @file circle_algorithm.cpp
 * @author Michael Wittmann (miw2006@thi.de)
 * @brief Implementation of the methods of Circle Algorithm class.
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

#include "ros2_collision_detection/circle_algorithm.hpp"
#include <algorithm>
#include <cmath>
#include <sstream>
#include <vector>

#define POLYNOMIAL_ARRAY_LENGTH 5  // quartic equation has variable of degree 0 to 4

namespace ros2_collision_detection_plugins {

CircleAlgorithm::CircleAlgorithm() {
    // Initialize ROS2 logging
    RCLCPP_INFO(rclcpp::get_logger("CircleAlgorithm"), "CircleAlgorithm::CircleAlgorithm constructed.");
}

std::string CircleAlgorithm::convertMotionStructToString(const object_motion_t &object_motion) {
    std::stringstream result;
    result << "center_pos_x = " << object_motion.center_pos_x << std::endl;
    result << "center_pos_y = " << object_motion.center_pos_y << std::endl;
    result << "length = " << object_motion.length << std::endl;
    result << "width = " << object_motion.width << std::endl;
    result << "heading = " << object_motion.heading << std::endl;
    result << "speed = " << object_motion.speed << std::endl;
    result << "acceleration = " << object_motion.acceleration << std::endl;

    return result.str();
}

double CircleAlgorithm::computeSinFromHeading(const float &heading) {
    return sin(heading * M_PI / 180.0);
}

double CircleAlgorithm::computeCosFromHeading(const float &heading) {
    return cos(heading * M_PI / 180.0);
}

double CircleAlgorithm::computeHeadingAdjustedValue(const float &value_to_adjust, const double &trigonometric_value) {
    return trigonometric_value * value_to_adjust;
}

double CircleAlgorithm::computeRadiusFromLength(const float &length, const float &width) {
    return sqrt(length * length + width * width) / 2;
}

std::optional<double> CircleAlgorithm::calculateTTC(const object_motion_t &subject_object_motion, const object_motion_t &perceived_object_motion) {
    std::optional<double> ttc_optional;

    RCLCPP_DEBUG(rclcpp::get_logger("CircleAlgorithm"), "subject object motion: \n%s", convertMotionStructToString(subject_object_motion).c_str());
    RCLCPP_DEBUG(rclcpp::get_logger("CircleAlgorithm"), "perceived object motion: \n%s", convertMotionStructToString(perceived_object_motion).c_str());

    if (subject_object_motion.length <= 0 || subject_object_motion.width <= 0 || perceived_object_motion.length <= 0 || perceived_object_motion.width <= 0) {
        RCLCPP_ERROR(rclcpp::get_logger("CircleAlgorithm"), "CircleAlgorithm::calculateTTC: length or width is not allowed to be zero or lower.");
        return ttc_optional;
    }

    if (subject_object_motion.heading < 0 || subject_object_motion.heading > 360 || perceived_object_motion.heading < 0 || perceived_object_motion.heading > 360) {
        RCLCPP_ERROR(rclcpp::get_logger("CircleAlgorithm"), "CircleAlgorithm::calculateTTC: heading value must be in range [0; 360].");
        return ttc_optional;
    }

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

    double radius_subject_obj = computeRadiusFromLength(subject_object_motion.length, subject_object_motion.width);
    double radius_perceived_obj = computeRadiusFromLength(perceived_object_motion.length, perceived_object_motion.width);

    double accel_diff_sin_adjusted = accel_subject_obj_sin_adjusted - accel_perceived_obj_sin_adjusted;
    double accel_diff_cos_adjusted = accel_subject_obj_cos_adjusted - accel_perceived_obj_cos_adjusted;
    double speed_diff_sin_adjusted = speed_subject_obj_sin_adjusted - speed_perceived_obj_sin_adjusted;
    double speed_diff_cos_adjusted = speed_subject_obj_cos_adjusted - speed_perceived_obj_cos_adjusted;
    double center_pos_x_diff = subject_object_motion.center_pos_x - perceived_object_motion.center_pos_x;
    double center_pos_y_diff = subject_object_motion.center_pos_y - perceived_object_motion.center_pos_y;

    double radius_sum = radius_subject_obj + radius_perceived_obj;

    double accel_diff_square_sin_adjusted = accel_diff_sin_adjusted * accel_diff_sin_adjusted;
    double accel_diff_square_cos_adjusted = accel_diff_cos_adjusted * accel_diff_cos_adjusted;
    double speed_diff_square_sin_adjusted = speed_diff_sin_adjusted * speed_diff_sin_adjusted;
    double speed_diff_square_cos_adjusted = speed_diff_cos_adjusted * speed_diff_cos_adjusted;
    double center_pos_x_diff_square = center_pos_x_diff * center_pos_x_diff;
    double center_pos_y_diff_square = center_pos_y_diff * center_pos_y_diff;

    double radius_sum_square = radius_sum * radius_sum;

    std::array<double, POLYNOMIAL_ARRAY_LENGTH> coefficients;
    coefficients[0] = circle_equation_solver::computeCoefficientForPowerZero(center_pos_x_diff_square, center_pos_y_diff_square, radius_sum_square);
    coefficients[1] = circle_equation_solver::computeCoefficientForPowerOne(speed_diff_sin_adjusted, speed_diff_cos_adjusted, center_pos_x_diff, center_pos_y_diff);
    coefficients[2] = circle_equation_solver::computeCoefficientForPowerTwo(accel_diff_sin_adjusted, accel_diff_cos_adjusted, speed_diff_square_sin_adjusted, speed_diff_square_cos_adjusted, center_pos_x_diff, center_pos_y_diff);
    coefficients[3] = circle_equation_solver::computeCoefficientForPowerThree(accel_diff_sin_adjusted, accel_diff_cos_adjusted, speed_diff_sin_adjusted, speed_diff_cos_adjusted);
    coefficients[4] = circle_equation_solver::computeCoefficientForPowerFour(accel_diff_square_sin_adjusted, accel_diff_square_cos_adjusted);

    std::vector<double> real_positive_roots = circle_equation_solver::solvePolynomialEquationGSL(coefficients);

    if (real_positive_roots.empty()) {
        RCLCPP_DEBUG(rclcpp::get_logger("CircleAlgorithm"), "CircleAlgorithm::calculateTTC: no real positive roots found.");
        return ttc_optional;
    }
    
    ttc_optional = *std::min_element(real_positive_roots.begin(), real_positive_roots.end());
    return ttc_optional;
}

}  // namespace ros2_collision_detection_plugins


/* 
#include "ros2_collision_detection/circle_algorithm.hpp"
#include <algorithm>
#include <cmath>
#include <sstream>
#include <vector>

#define POLYNOMIAL_ARRAY_LENGTH 5  // quartic equation has variable of degree 0 to 4

CircleAlgorithm::CircleAlgorithm() {
    // Initialize ROS2 logging
    RCLCPP_INFO(rclcpp::get_logger("CircleAlgorithm"), "CircleAlgorithm::CircleAlgorithm constructed.");
}

std::string CircleAlgorithm::convertMotionStructToString(const object_motion_t &object_motion) {
    std::stringstream result;
    result << "center_pos_x = " << object_motion.center_pos_x << std::endl;
    result << "center_pos_y = " << object_motion.center_pos_y << std::endl;
    result << "length = " << object_motion.length << std::endl;
    result << "width = " << object_motion.width << std::endl;
    result << "heading = " << object_motion.heading << std::endl;
    result << "speed = " << object_motion.speed << std::endl;
    result << "acceleration = " << object_motion.acceleration << std::endl;

    return result.str();
}

double CircleAlgorithm::computeSinFromHeading(const float &heading) {
    return sin(heading * M_PI / 180.0);
}

double CircleAlgorithm::computeCosFromHeading(const float &heading) {
    return cos(heading * M_PI / 180.0);
}

double CircleAlgorithm::computeHeadingAdjustedValue(const float &value_to_adjust, const double &trigonometric_value) {
    return trigonometric_value * value_to_adjust;
}

double CircleAlgorithm::computeRadiusFromLength(const float &length, const float &width) {
    return sqrt(length * length + width * width) / 2;
}

std::optional<double> CircleAlgorithm::calculateTTC(const object_motion_t &subject_object_motion, const object_motion_t &perceived_object_motion) {
    std::optional<double> ttc_optional;

    RCLCPP_DEBUG(rclcpp::get_logger("CircleAlgorithm"), "subject object motion: \n%s", convertMotionStructToString(subject_object_motion).c_str());
    RCLCPP_DEBUG(rclcpp::get_logger("CircleAlgorithm"), "perceived object motion: \n%s", convertMotionStructToString(perceived_object_motion).c_str());

    if (subject_object_motion.length <= 0 || subject_object_motion.width <= 0 || perceived_object_motion.length <= 0 || perceived_object_motion.width <= 0) {
        RCLCPP_ERROR(rclcpp::get_logger("CircleAlgorithm"), "CircleAlgorithm::calculateTTC: length or width is not allowed to be zero or lower.");
        return ttc_optional;
    }

    if (subject_object_motion.heading < 0 || subject_object_motion.heading > 360 || perceived_object_motion.heading < 0 || perceived_object_motion.heading > 360) {
        RCLCPP_ERROR(rclcpp::get_logger("CircleAlgorithm"), "CircleAlgorithm::calculateTTC: heading value must be in range [0; 360].");
        return ttc_optional;
    }

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

    double radius_subject_obj = computeRadiusFromLength(subject_object_motion.length, subject_object_motion.width);
    double radius_perceived_obj = computeRadiusFromLength(perceived_object_motion.length, perceived_object_motion.width);

    double accel_diff_sin_adjusted = accel_subject_obj_sin_adjusted - accel_perceived_obj_sin_adjusted;
    double accel_diff_cos_adjusted = accel_subject_obj_cos_adjusted - accel_perceived_obj_cos_adjusted;
    double speed_diff_sin_adjusted = speed_subject_obj_sin_adjusted - speed_perceived_obj_sin_adjusted;
    double speed_diff_cos_adjusted = speed_subject_obj_cos_adjusted - speed_perceived_obj_cos_adjusted;
    double center_pos_x_diff = subject_object_motion.center_pos_x - perceived_object_motion.center_pos_x;
    double center_pos_y_diff = subject_object_motion.center_pos_y - perceived_object_motion.center_pos_y;

    double radius_sum = radius_subject_obj + radius_perceived_obj;

    double accel_diff_square_sin_adjusted = accel_diff_sin_adjusted * accel_diff_sin_adjusted;
    double accel_diff_square_cos_adjusted = accel_diff_cos_adjusted * accel_diff_cos_adjusted;
    double speed_diff_square_sin_adjusted = speed_diff_sin_adjusted * speed_diff_sin_adjusted;
    double speed_diff_square_cos_adjusted = speed_diff_cos_adjusted * speed_diff_cos_adjusted;
    double center_pos_x_diff_square = center_pos_x_diff * center_pos_x_diff;
    double center_pos_y_diff_square = center_pos_y_diff * center_pos_y_diff;

    double radius_sum_square = radius_sum * radius_sum;

    std::array<double, POLYNOMIAL_ARRAY_LENGTH> coefficients;
    coefficients[0] = circle_equation_solver::computeCoefficientForPowerZero(center_pos_x_diff_square, center_pos_y_diff_square, radius_sum_square);
    coefficients[1] = circle_equation_solver::computeCoefficientForPowerOne(speed_diff_sin_adjusted, speed_diff_cos_adjusted, center_pos_x_diff, center_pos_y_diff);
    coefficients[2] = circle_equation_solver::computeCoefficientForPowerTwo(accel_diff_sin_adjusted, accel_diff_cos_adjusted, speed_diff_square_sin_adjusted, speed_diff_square_cos_adjusted, center_pos_x_diff, center_pos_y_diff);
    coefficients[3] = circle_equation_solver::computeCoefficientForPowerThree(accel_diff_sin_adjusted, accel_diff_cos_adjusted, speed_diff_sin_adjusted, speed_diff_cos_adjusted);
    coefficients[4] = circle_equation_solver::computeCoefficientForPowerFour(accel_diff_square_sin_adjusted, accel_diff_square_cos_adjusted);

    std::vector<double> real_positive_roots = circle_equation_solver::solvePolynomialEquationGSL(coefficients);

    if (real_positive_roots.empty()) {
        RCLCPP_DEBUG(rclcpp::get_logger("CircleAlgorithm"), "CircleAlgorithm::calculateTTC: no real positive roots found.");
        return ttc_optional;
    }
    
    ttc_optional = *std::min_element(real_positive_roots.begin(), real_positive_roots.end());
    return ttc_optional;
}

*/