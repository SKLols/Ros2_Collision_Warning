#include "ros2_collision_detection/ttc_algorithm.hpp"
//#include "ros2_collision_detection/circle_algorithm.hpp"
#include "ros2_collision_detection/n_circle_algorithm.hpp"
#include <cmath>


#define POLYNOMIAL_ARRAY_LENGTH 5   //!< quartic equation has variable of degree 0 to 4

namespace ros2_collision_detection_plugins {

    class CircleAlgorithm : public ros2_collision_detection::TTCAlgorithm
    {
        public:
            CircleAlgorithm()
            {
                //RCLCPP_INFO("CircleAlgorithm constructed")
                printf("Constructor created");
            }

            void initialize(double side_length) override 
            {
                side_length_ = side_length;
            }
            //void init(parameter_map_t &parameter_map) override
            //{
            //    parameter_map_=parameter_map
            //}

            double area() override 
            {
                return side_length_*side_length_;
            }

            double computeCoefficientForPowerFour(double &accel_diff_sq_sin_adj, double &accel_diff_sq_cos_adj)
            {
                return (accel_diff_sq_sin_adj + accel_diff_sq_cos_adj) / 4;
            }

            protected:
                double side_length_;
            //std::optional<double> calculateTTC(
            //const object_motion_t &subject_object_motion,
            //const object_motion_t &perceived_object_motion
            //) override
            //{
            //    return 
            //}
    };

    class CircleEquationSolver : public ros2_collision_detection::TTCAlgorithm //Just change class name as per header file
    {
        public:
            CircleEquationSolver() //Just change to class name as per header file
            {
                //RCLCPP_INFO("CircleAlgorithm constructed")
                printf("Constructor 2 created");
            }

            /**
            * @brief Compute the coefficient for the 4th power of variable TTC.
            * 
            * Compute the coefficient for variable TTC with exponent 4 using an rearranged version of the equation for the Circle Algorithm.
            * The coefficient is computed following the formula: 
            * [ (sin(alpha) * a_i - sin(beta) * a_j)^2 + (cos(alpha) * a_i - cos(beta) * a_j)^2 ] * 0.25 
            * 
            * @param accel_diff_sq_sin_adj The squared difference between the sin-adjusted accelerations of object i and object j. 
            * @param accel_diff_sq_cos_adj The squared difference between the cos-adjusted accelerations of object i and object j.
            * @return The coefficient for the 4th power of variable TTC.
            */
            double computeCoefficientForPowerFour(double &accel_diff_sq_sin_adj, double &accel_diff_sq_cos_adj)
            {
                return (accel_diff_sq_sin_adj + accel_diff_sq_cos_adj) / 4;
            }

            void initialize(double side_length) override 
            {
                side_length_ = side_length;
            }
            //void init(parameter_map_t &parameter_map) override
            //{
            //    parameter_map_=parameter_map
            //}

            double area() override 
            {
                return side_length_*side_length_*side_length_;
            }

            protected:
                double side_length_;
            //std::optional<double> calculateTTC(
            //const object_motion_t &subject_object_motion,
            //const object_motion_t &perceived_object_motion
            //) override
            //{
            //    return 
            //}
    };

} // namespace ros2_collision_detection_plugins


#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(ros2_collision_detection_plugins::CircleAlgorithm, ros2_collision_detection::TTCAlgorithm)
PLUGINLIB_EXPORT_CLASS(ros2_collision_detection_plugins::CircleEquationSolver, ros2_collision_detection::TTCAlgorithm)


//namespace ros2_collision_detection {
//
//}
//
//#include <pluginlib/class_list_macros.hpp>
//
//
//PLUGINLIB_EXPORT_CLASS(ros2_collision_detection_plugins::CircleAlgorithm, ros2_collision_detection::TTCAlgorithm)
//PLUGINLIB_EXPORT_CLASS(ros2_collision_detection_plugins::NCircleAlgorithm, ros2_collision_detection::TTCAlgorithm)
