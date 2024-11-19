#include "ros2_collision_detection/ttc_algorithm.hpp"
//#include "ros2_collision_detection/circle_algorithm.hpp"
#include "ros2_collision_detection/n_circle_algorithm.hpp"
#include <cmath>

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
