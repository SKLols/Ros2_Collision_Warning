/**
 * @file ttc_algorithm.h
 * @author Michael Wittmann (miw2006@thi.de)
 * @brief Header for TTC Algorithm interface.
 * @version 0.1
 * @date 2022-04-22
 * 
 * @copyright Copyright (c) 2022
 * 
 * @author Sourabh Lolge (sol9471@thi.de)
 * @date 2024-10-31
 * @brief Updated from ROS 1 to ROS 2.
 */

#ifndef _TTC_ALGORITHM_HPP_
#define _TTC_ALGORITHM_HPP_

#include <map>
#include <optional>  // For std::optional
#include <variant>   // For std::variant
#include <string>    // For std::string
#include <gsl/gsl_errno.h>
#include <gsl/gsl_poly.h>

namespace ros2_collision_detection {

    // Type for parameter map using std::variant
    using parameter_map_t = std::map<std::string, std::variant<int, std::string>>;

    // Struct for object motion
    struct object_motion_t {
        float center_pos_x;
        float center_pos_y;
        float length;
        float width;
        float heading;
        float speed;
        float acceleration;
    };

    // Base class for TTC Algorithm
    class TTCAlgorithm
    {
    public:
        virtual void initialize(double side_length) = 0;
        virtual double area() = 0;
        virtual double computeCoefficientForPowerFour(double &accel_diff_sq_sin_adj, double &accel_diff_sq_cos_adj) = 0;
        //virtual void init(parameter_map_t &parameter_map) = 0;
        //virtual std::optional<double> calculateTTC(
        //    const object_motion_t &subject_object_motion,
        //    const object_motion_t &perceived_object_motion
        //) = 0;
        virtual ~TTCAlgorithm() = default; // Virtual destructor

    protected:
        TTCAlgorithm(){}
    };

    class CircleAlgorithm : public TTCAlgorithm
    {
    public:
        CircleAlgorithm();
        
        void initialize(double side_length) override {};
        //void init(parameter_map_t &parameter_map) override
        //{
        //    // Initialize circle-specific parameters
        //};

        double area() override;

        double computeCoefficientForPowerFour(double &accel_diff_sq_sin_adj, double &accel_diff_sq_cos_adj);
        //std::optional<double> calculateTTC(
        //    const object_motion_t &subject_object_motion,
        //    const object_motion_t &perceived_object_motion
        //) override;
        //{
        //    // Implement the calculation logic for Circle Algorithm
        //    return std::optional<double>(42.0); // Placeholder value
        //}
    };

    class CircleEquationSolver : public TTCAlgorithm //Just change class name
    {
    public:
        CircleEquationSolver(); //Just change this to class name
        
        void initialize(double side_length) override {};
        //void init(parameter_map_t &parameter_map) override
        //{
        //    // Initialize circle-specific parameters
        //};

        double area() override;

        double computeCoefficientForPowerFour(double &accel_diff_sq_sin_adj, double &accel_diff_sq_cos_adj);
        //std::optional<double> calculateTTC(
        //    const object_motion_t &subject_object_motion,
        //    const object_motion_t &perceived_object_motion
        //) override;
        //{
        //    // Implement the calculation logic for Circle Algorithm
        //    return std::optional<double>(42.0); // Placeholder value
        //}
    };

} // namespace ros2_collision_detection

#endif //_TTC_ALGORITHM_HPP_






//#ifndef ROS2_COLLISION_DETECTION_TTC_ALGORITHM_HPP
//#define ROS2_COLLISION_DETECTION_TTC_ALGORITHM_HPP
//
//#include <map>
//#include <variant> //std::variant from the C++ Standard Library (introduced in C++17) is used instead of boost::variant.
//#include <optional> // Use std::optional instead of boost::optional
//#include <pluginlib/class_list_macros.hpp> //Must for ROS 2 as plugin library is there in ros2 
//
//namespace ros2_collision_detection {
//
///**
// * @brief Map that includes the parameters for TTC Algorithm initialization.
// * 
// */
//
//typedef std::map<std::string, std::variant<int, std::string>> parameter_map_t;
//
///**
// * @brief Struct that represents an object's motion.
// * 
// */
//typedef struct {
//    float center_pos_x; //!< The object's center x-coordinate.
//    float center_pos_y; //!< The object's center y-coordinate.
//    float length;       //!< The longer side of the object.
//    float width;        //!< The shorter side of the object.
//    float heading;      //!< The object's heading.
//    float speed;        //!< The object's speed.
//    float acceleration; //!< The object's acceleration.
//} object_motion_t;
//
//
///**
// * @brief Interface that defines the TTC calculation method that all concrete TTC Algorithm classes must implement. 
// * 
// */
//class TTCAlgorithm
//{
//public:
//    /**
//     * @brief Initialize the TTC Algorithm with the passed parameters.
//     * 
//     * @param parameter_map A key-value map containing parameter names and parameter values.
//     */
//    virtual void init(parameter_map_t &parameter_map) = 0;
//
//    /**
//     * @brief Calculate the Time-To-Collision between the Subject Object Motion and the Perceived Object Motion.
//     * 
//     * @param subject_object_motion Object Motion struct representing the Subject Object Motion.
//     * @param perceived_object_motion Object Motion struct representing the Perceived Object Motion.
//     * @return Optional that either contains a valid Time-To-Collision or has no valid content.
//     */
//    virtual std::optional<double> calculateTTC(
//        const object_motion_t &subject_object_motion,
//        const object_motion_t &perceived_object_motion
//    ) = 0; 
//};
//
//} // namespace ros2_collision_detection
//
//#endif // ROS2_COLLISION_DETECTION_TTC_ALGORITHM_HPP