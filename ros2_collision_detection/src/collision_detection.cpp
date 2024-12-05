#include "rclcpp/rclcpp.hpp"
#include "v2xvf_interfaces/msg/collision_check_result.hpp"
#include "v2xvf_interfaces/msg/object_movement.hpp"
#include "v2xvf_interfaces/msg/perceived_object_motion.hpp"
#include "v2xvf_interfaces/msg/perceived_objects.hpp"
#include "v2xvf_interfaces/msg/subject_vehicle_motion.hpp"

// definition of default values for launch parameters
#define DEFAULT_PUBLISH_TOPIC "/collision_warning"
#define DEFAULT_TTC_ALGORITHM_CLASSNAME "CircleAlgorithm"
#define DEFAULT_WARNING_GENERATOR_ALGORITHM_CLASSNAME "TTCOnlyWarningAlgorithm"
#define DEFAULT_TTC_ALGORITHM_CIRCLE_COUNT 1
#define DEFAULT_SUBJECT_VEHICLE_LENGTH 5
#define DEFAULT_SUBJECT_VEHICLE_WIDTH 1.8

class CollisionDetection : public rclcpp::Node
{
    public:
        CollisionDetection()
         : Node ("collision_detection")
          //ttc_algorithm_loader("ros2_collision_detection", "TTCAlgorithm"),
          //warning_generator_algorithm_loader("ros2_collision_detection", "WarningGeneratorAlgorithm"),
          //approximate_synchronizer(ApproximateSyncPolicy(10), fused_objects_subscriber, ego_position_subscriber),
          //warning_generator(collision_warning_publisher)
        {
           RCLCPP_INFO(nh->get_logger(), "Node successfully initialized-0.");
           CollisionDetection::init(); //Publishers and subscribers
        }

        void init()
        {
            //main logic
        }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc,argv); //initialise ros2

    auto collision_detection_node = std::make_shared<CollisionDetection>();

    rclcpp::spin(collision_detection_node);
    rclcpp::shutdown();
    return 0;
}