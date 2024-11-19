#include <cstdio>
#include <pluginlib/class_loader.hpp>
#include <ros2_collision_detection/ttc_algorithm.hpp>
#include "rclcpp/rclcpp.hpp"
#include "v2xvf_interfaces/msg/collision_check_result.hpp"
#include "v2xvf_interfaces/msg/object_movement.hpp"
#include "v2xvf_interfaces/msg/perceived_object_motion.hpp"
#include "v2xvf_interfaces/msg/perceived_objects.hpp"
#include "v2xvf_interfaces/msg/subject_vehicle_motion.hpp"
#include <boost/shared_ptr.hpp>
#include <gsl/gsl_errno.h>
#include <gsl/gsl_poly.h>

// definition of default values for launch parameters
#define DEFAULT_PUBLISH_TOPIC "/collision_warning"
#define DEFAULT_TTC_ALGORITHM_CLASSNAME "CircleAlgorithm"
#define DEFAULT_WARNING_GENERATOR_ALGORITHM_CLASSNAME "TTCOnlyWarningAlgorithm"
#define DEFAULT_TTC_ALGORITHM_CIRCLE_COUNT 1
#define DEFAULT_SUBJECT_VEHICLE_LENGTH 5
#define DEFAULT_SUBJECT_VEHICLE_WIDTH 1.8

//class CollisionDetection : public rclcpp::Node
//{
//    public:
//        CollisionDetection()
//         : Node ("collision_detection")
//          //ttc_algorithm_loader("ros2_collision_detection", "TTCAlgorithm"),
//          //warning_generator_algorithm_loader("ros2_collision_detection", "WarningGeneratorAlgorithm"),
//          //approximate_synchronizer(ApproximateSyncPolicy(10), fused_objects_subscriber, ego_position_subscriber),
//          //warning_generator(collision_warning_publisher)
//        {
//           CollisionDetection::init(); //Publishers and subscribers
//        }
//
//        void init()
//        {
//            //main logic
//        }
//};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);  // Initialize ROS 2

    // Load the plugin using ClassLoader
    pluginlib::ClassLoader<ros2_collision_detection::TTCAlgorithm> poly_loader("ros2_collision_detection", "ros2_collision_detection::TTCAlgorithm");

    try {
        // Load the plugin (circle algorithm in this case)
        std::shared_ptr<ros2_collision_detection::TTCAlgorithm> circle_algorithm = poly_loader.createSharedInstance("ros2_collision_detection_plugins::CircleAlgorithm");
        circle_algorithm->initialize(10.0);

        std::shared_ptr<ros2_collision_detection::TTCAlgorithm> circle_equation_solver = poly_loader.createSharedInstance("ros2_collision_detection_plugins::CircleEquationSolver");
        circle_equation_solver->initialize(10.0);

        double accel_diff_sq_sin_adj = 2.0;  // Example value
        double accel_diff_sq_cos_adj = 4.0;  // Example value

        double result = circle_algorithm->computeCoefficientForPowerFour(accel_diff_sq_sin_adj, accel_diff_sq_cos_adj);
        double result_equation = circle_equation_solver->computeCoefficientForPowerFour(accel_diff_sq_sin_adj, accel_diff_sq_cos_adj);

        // Create and populate the parameter_map_t
        //ros2_collision_detection::parameter_map_t parameter_map;
        //parameter_map["side_length"] = 10;  // Example: Side length for CircleAlgorithm

        // Pass the populated map to init
        //plugin->init(parameter_map);

        printf("Circle area: %.2f\n", circle_algorithm->area());
        printf("Circle Equation Solver area: %.2f\n", circle_equation_solver->area());
        printf("Result of computeCoefficientForPowerFour: %.2f\n", result);
        printf("Result of equation computeCoefficientForPowerFour: %.2f\n", result_equation);
        // Test the plugin's functionality
        //ros2_collision_detection::object_motion_t subject = {0, 0, 10, 5, 0, 20, 0};
        //ros2_collision_detection::object_motion_t perceived = {10, 10, 10, 5, 0, 20, 0};
        //auto ttc = plugin->calculateTTC(subject, perceived);
        //if (ttc) {
        //    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "TTC: %f", *ttc);

        
        }

    catch (pluginlib::PluginlibException& ex) {
        printf("The plugin failed to load for some reason. Error: %s\n", ex.what());
        //RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to load plugin: %s", ex.what());
    }

    rclcpp::shutdown();
    return 0;
    
}





//class CollisionDetection : public rclcpp::Node
//{
//    public:
//        CollisionDetection()
//         : Node ("collision_detection")
//          //ttc_algorithm_loader("ros2_collision_detection", "TTCAlgorithm"),
//          //warning_generator_algorithm_loader("ros2_collision_detection", "WarningGeneratorAlgorithm"),
//          //approximate_synchronizer(ApproximateSyncPolicy(10), fused_objects_subscriber, ego_position_subscriber),
//          //warning_generator(collision_warning_publisher)
//        {
//           CollisionDetection::init(); //Publishers and subscribers
//        }
//
//        void init()
//        {
//            //main logic
//        }
//};
//
//int main(int argc, char ** argv)
//{
//    rclcpp::init(argc,argv); //initialise ros2
//
//    auto collision_detection_node = std::make_shared<CollisionDetection>();
//
//    printf("initial setup successfully.\n");
//
//    pluginlib::ClassLoader<ros2_collision_detection::TTCAlgorithm> poly_loader("ros2_collision_detection", "ros2_collision_detection::TTCAlgorithm");
//  
//    try 
//    {
//      // Try to load the CircleAlgorithm plugin
//      std::shared_ptr<ros2_collision_detection::TTCAlgorithm> plugin = poly_loader.createSharedInstance("ros2_collision_detection_plugins::CircleAlgorithm");
//      printf("CircleAlgorithm plugin loaded successfully.");
//    } 
//    catch (pluginlib::PluginlibException& ex) 
//    {
//      printf("Failed to load plugin: %s \n", ex.what());
//    }
//
//    rclcpp::spin(collision_detection_node);
//    rclcpp::shutdown();
//
//    return 0;
//}
//