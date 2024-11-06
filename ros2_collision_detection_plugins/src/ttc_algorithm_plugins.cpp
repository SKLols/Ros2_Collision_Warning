#include "ros2_collision_detection/ttc_algorithm.hpp"


namespace ros2_collision_detection_plugins
{
    class CircleAlgorithm : public TTCAlgorithm
    {
        public:
        CircleAlgorithm()
        {

        }

        virtual void init(parameter_map_t &parameter_map) override {

        }  // Add your method signature

        // Calculate Time-To-Collision for CircleAlgorithm
        virtual std::optional<double> calculateTTC(
            const object_motion_t &subject_object_motion,
            const object_motion_t &perceived_object_motion) override {
            // Implement the TTC calculation logic here
            return std::nullopt;  // Return a dummy value for now
        }
    };

    class NCircleAlgorithm : public TTCAlgorithm 
    {
        public:
        NCircleAlgorithm() {
            // Constructor logic, if necessary
        }

        // Initialize method for NCircleAlgorithm
        virtual void init(parameter_map_t &parameter_map) override {
            // Implement initialization logic here
        }

        // Calculate Time-To-Collision for NCircleAlgorithm
        virtual std::optional<double> calculateTTC(
            const object_motion_t &subject_object_motion,
            const object_motion_t &perceived_object_motion) override {
            // Implement the TTC calculation logic here
            return std::nullopt;  // Return a dummy value for now
        }
    };
//TtcAlgorithmPlugins::TtcAlgorithmPlugins()
//{
//}

//TtcAlgorithmPlugins::~TtcAlgorithmPlugins()
//{
//}

}  // namespace ros2_collision_detection_plugins

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(ros2_collision_detection_plugins::CircleAlgorithm, TTCAlgorithm)
PLUGINLIB_EXPORT_CLASS(ros2_collision_detection_plugins::NCircleAlgorithm, TTCAlgorithm)
