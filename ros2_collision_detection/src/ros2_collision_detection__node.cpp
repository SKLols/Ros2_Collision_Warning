/**
 * @file collision_detection.h
 * @author Michael Wittmann (miw2006@thi.de)
 * @brief Header for Collision Detection class.
 * @version 0.1
 * @date 2022-04-13
 * 
 * @copyright Copyright (c) 2022
 * 
 * @author Sourabh Lolge (sol9471@thi.de)
 * @date 2024-11-29
 * @brief Updated from ROS 1 to ROS 2.
 * 
 */

#include <ros2_collision_detection/ros2_collision_detection__node.hpp>
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


//#include <ros2_collision_detection/enum_result_type.hpp>
//#include "ros2_collision_detection/warning_generator_algorithm.hpp"
//#include "ros2_collision_detection/warning_generator.hpp"
//#include "ros2_collision_detection/ttc_only_warning_algorithm.hpp"
//#include "ros2_collision_detection/ttc_calculator.hpp"


// definition of default values for launch parameters_
#define DEFAULT_PUBLISH_TOPIC "/collision_warning"
#define DEFAULT_TTC_ALGORITHM_CLASSNAME "ros2_collision_detection_plugins::CircleAlgorithm"
#define DEFAULT_WARNING_GENERATOR_ALGORITHM_CLASSNAME "TTCOnlyWarningAlgorithm"
#define DEFAULT_TTC_ALGORITHM_CIRCLE_COUNT 1
#define DEFAULT_SUBJECT_VEHICLE_LENGTH 5
#define DEFAULT_SUBJECT_VEHICLE_WIDTH 1.8

CollisionDetection::CollisionDetection(std::shared_ptr<rclcpp::Node> nh) 
:ttc_algorithm_loader("ros2_collision_detection", "ros2_collision_detection::TTCAlgorithm"),
warning_generator_algorithm_loader("ros2_collision_detection", "WarningGeneratorAlgorithm"),
//approximate_synchronizer(ApproximateSyncPolicy(10), fused_objects_subscriber, ego_position_subscriber),
warning_generator(collision_warning_publisher)
{
    RCLCPP_INFO(nh->get_logger(), "Node successfully initialized-0.");
    node_handle = nh;
    CollisionDetection::init();
}


void CollisionDetection::init()
{
    RCLCPP_INFO(node_handle->get_logger(), "Node successfully initialized-2.");

    // initialize the components with launch parameter values
    initFromLaunchParameters();

    RCLCPP_INFO(node_handle->get_logger(), "Node successfully initialized-3.");

    // register callback from ttc_calculator to warning_generator
    ttc_calculator.addWarningSignalCallback(boost::bind(&WarningGenerator::createWarning, &warning_generator, _1, _2, _3)); 
    RCLCPP_INFO(node_handle->get_logger(), "Node successfully initialized-4.");
    //@todo
    collision_warning_publisher = node_handle->create_publisher<v2xvf_interfaces::msg::CollisionCheckResult>("/collision_warning", 10);
    fused_objects_subscriber.subscribe(node_handle, "/fused_objects");
    ego_position_subscriber.subscribe(node_handle, "/ego_position");
    //approximate_synchronizer.registerCallback(boost::bind(&CollisionDetection::callback, this, _1, _2));
    
    // log successful init
    RCLCPP_INFO(node_handle->get_logger(),"collision_detection node successfully initialized.");
}

//initFromLaunchParameters function Completed
void CollisionDetection::initFromLaunchParameters()
{
    // Check launch parameters and set defaults if necessary
    checkLaunchParameters();

    // load the concrete TTC Algorithm and Warning Generator classes
    loadPlugins();

    // initialize TTC Calculator and Warning Generator components with launch parameters
    initComponents();
}

//checkLaunchParameters function Completed
void CollisionDetection::checkLaunchParameters()
{
    if (!node_handle->has_parameter("publish_topic"))
    {
        // Default value for param "publish_topic" is DEFAULT_PUBLISH_TOPIC
        node_handle->declare_parameter("publish_topic", DEFAULT_PUBLISH_TOPIC);
        RCLCPP_WARN(node_handle->get_logger(),"CollisionDetection::loadPlugins: using default value for 'publish_topic'.");
    }

    if (!node_handle->has_parameter("ttc_algorithm_classname"))
    {
        // Default value for param "ttc_algorithm_classname" is DEFAULT_TTC_ALGORITHM_CLASSNAME
        node_handle->declare_parameter("ttc_algorithm_classname", DEFAULT_TTC_ALGORITHM_CLASSNAME);
        RCLCPP_WARN(node_handle->get_logger(), "CollisionDetection::loadPlugins: using default value for 'ttc_algorithm_classname'.");
    }

    if (!node_handle->has_parameter("warning_generator_algorithm_classname"))
    {
        // Default value for param "warning_generator_algorithm_classname" is DEFAULT_WARNING_GENERATOR_ALGORITHM_CLASSNAME
        node_handle->declare_parameter("warning_generator_algorithm_classname", DEFAULT_WARNING_GENERATOR_ALGORITHM_CLASSNAME);
        RCLCPP_WARN(node_handle->get_logger(),"CollisionDetection::loadPlugins: using default value for 'warning_generator_algorithm_classname'.");
    }

    if (!node_handle->has_parameter("ttc_algorithm_circle_count"))
    {
        // Default value for param "ttc_algorithm_circle_count" is DEFAULT_TTC_ALGORITHM_CIRCLE_COUNT
        node_handle->declare_parameter("ttc_algorithm_circle_count", DEFAULT_TTC_ALGORITHM_CIRCLE_COUNT);
        RCLCPP_WARN(node_handle->get_logger(),"CollisionDetection::loadPlugins: using default value for 'ttc_algorithm_circle_count'.");
    }

    if (!node_handle->has_parameter("subject_vehicle_length"))
    {
        // Default value for param "subject_vehicle_length" is DEFAULT_SUBJECT_VEHICLE_LENGTH
        node_handle->declare_parameter("subject_vehicle_length", DEFAULT_SUBJECT_VEHICLE_LENGTH);
        RCLCPP_WARN(node_handle->get_logger(),"CollisionDetection::loadPlugins: using default value for 'subject_vehicle_length'.");
    }

    if (!node_handle->has_parameter("subject_vehicle_width"))
    {
        // Default value for param "subject_vehicle_width" is DEFAULT_SUBJECT_VEHICLE_WIDTH
        node_handle->declare_parameter("subject_vehicle_width", DEFAULT_SUBJECT_VEHICLE_WIDTH);
        RCLCPP_WARN(node_handle->get_logger(),"CollisionDetection::loadPlugins: using default value for 'subject_vehicle_width'.");
    }    

}

void CollisionDetection::loadPlugins()
{
    std::string ttc_algorithm_classname; //!< The name of the TTC Algorithm class to load
    int ttc_algorithm_circle_count;      //!< The number of circles that should be used to represent a vehicle
    ros2_collision_detection::parameter_map_t param_map;           //!< The parameters that are passed to the init method of the TTC Algorithm

    if(node_handle->get_parameter("ttc_algorithm_classname", ttc_algorithm_classname))
    {
        if(node_handle->get_parameter("ttc_algorithm_circle_count", ttc_algorithm_circle_count))
        {
            param_map.insert({"ttc_algorithm_circle_count", std::variant<int, std::string>(ttc_algorithm_circle_count)}); 

            try
            {
                std::shared_ptr<ros2_collision_detection::TTCAlgorithm> ttc_algorithm_ptr = ttc_algorithm_loader.createSharedInstance(ttc_algorithm_classname);
                ttc_algorithm_ptr->initialize(param_map); // init before shared pointer ownership changes
                ttc_calculator.setTTCAlgorithm(ttc_algorithm_ptr);
            }
            catch(pluginlib::PluginlibException& e)
            {
                RCLCPP_FATAL(node_handle->get_logger(),"CollisionDetection::loadPlugins: cannot load TTC Algorithm plugin: %s", e.what());
                rclcpp::shutdown();
                exit(0);
            }

        }
        else
        {
            RCLCPP_FATAL(node_handle->get_logger(),"CollisionDetection::loadPlugins: parameter 'ttc_algorithm_circle_count' could not be retrieved.");
            rclcpp::shutdown();
            exit(0);   
        }

    }
    else
    {
        RCLCPP_FATAL(node_handle->get_logger(),"CollisionDetection::loadPlugins: parameter 'ttc_algorithm_classname' could not be retrieved.");
        rclcpp::shutdown();
        exit(0);
    }

    std::string warning_generator_algorithm_classname; //!< The name of the Warning Generator Algorithm class to load
    
    if(node_handle->get_parameter("warning_generator_algorithm_classname", warning_generator_algorithm_classname))
    {
        try
        {
            std::shared_ptr<WarningGeneratorAlgorithm> warning_generator_algorithm_ptr = warning_generator_algorithm_loader.createSharedInstance(warning_generator_algorithm_classname);
            warning_generator.setWarningGeneratorAlgorithm(warning_generator_algorithm_ptr);
        }
        catch(pluginlib::PluginlibException& e)
        {
            RCLCPP_FATAL(node_handle->get_logger(),"CollisionDetection::loadPlugins: cannot load Warning Generator Algorithm plugin: %s", e.what());
            rclcpp::shutdown();
            exit(0);
        }
    }
    else
    {
        RCLCPP_FATAL(node_handle->get_logger(),"CollisionDetection::loadPlugins: parameter 'warning_generator_algorithm_classname' could not be retrieved.");
        rclcpp::shutdown();
        exit(0);
    }

}

//initComponents function Completed
void CollisionDetection::initComponents()
{
    std::string publish_topic_name;
    if(node_handle->get_parameter("publish_topic", publish_topic_name))
    {
        collision_warning_publisher = node_handle->create_publisher<v2xvf_interfaces::msg::CollisionCheckResult>(publish_topic_name, 10);
    }
    else
    {
        RCLCPP_FATAL(node_handle->get_logger(),"CollisionDetection::initComponents: parameter 'publish_topic' could not be retrieved.");
        rclcpp::shutdown();
        exit(0);
    }

    float subject_vehicle_length;   //!< The length of the subject vehicle
    float subject_vehicle_width;    //!< The width of the subject vehicle

    bool subject_vehicle_length_retrieved = node_handle->get_parameter("subject_vehicle_length", subject_vehicle_length);
    bool subject_vehicle_width_retrieved = node_handle->get_parameter("subject_vehicle_width", subject_vehicle_width);

    if(!subject_vehicle_length_retrieved)
    {
        RCLCPP_FATAL(node_handle->get_logger(),"CollisionDetection::initComponents: parameter 'subject_vehicle_length' could not be retrieved.");
        rclcpp::shutdown();
        exit(0);
    }

    if(!subject_vehicle_width_retrieved)
    {
        RCLCPP_FATAL(node_handle->get_logger(),"CollisionDetection::initComponents: parameter 'subject_vehicle_width' could not be retrieved.");
        rclcpp::shutdown();
        exit(0);
    }

    // dimensions of subject vehicle successfully retrieved
    ttc_calculator.setSubjectVehicleDimensions(subject_vehicle_length, subject_vehicle_width);
}

void CollisionDetection::callback(const v2xvf_interfaces::msg::PerceivedObjects::SharedPtr perceived_objects_msg, const v2xvf_interfaces::msg::SubjectVehicleMotion::SharedPtr subject_vehicle_motion_msg)
{
    // log seq number of the two messages
    //RCLCPP_DEBUG(node_handle->get_logger(),"CollisionDetection::callback: Subject vehicle msg: seq = %d | perceived object msg: seq = %d.", subject_vehicle_motion_msg->header.seq, perceived_objects_msg->header.seq);
    
    ttc_calculator.calculateAllTTCs(perceived_objects_msg, subject_vehicle_motion_msg);
}

//Main function Completed
int main(int argc, char **argv) 
{
    //Intialise ROS2 Node
    //ros::init(argc, argv, "collision_detecion",ros::init_options::AnonymousName); // create anonymous node if node if same name exists
    rclcpp::init(argc, argv);
    
    //Create anonymous node
    std::string node_name = "collision_detection";
    std::string unique_node_name = node_name + "_" + std::to_string(rand()); //to_string used to convert a value of fundam,ental type (integer, float) to its corresponding string representation

    //ros::NodeHandle nh;
    auto nh = std::make_shared<rclcpp::Node>(unique_node_name);

    //Log initialization
    RCLCPP_INFO(nh->get_logger(), "Node successfully initialized-1.");

    //CollisionDetection collision_detection(&nh);
    CollisionDetection collision_detection(nh);

    // Spin the node
    rclcpp::spin(nh);

    // Shutdown the ROS 2 system
    rclcpp::shutdown();
}

/*
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);  // Initialize ROS 2

    // Create a ROS 2 Node for logging
    auto node = std::make_shared<rclcpp::Node>("collision_detection_node");

    // Load the plugin using ClassLoader
    pluginlib::ClassLoader<ros2_collision_detection::TTCAlgorithm> poly_loader("ros2_collision_detection", "ros2_collision_detection::TTCAlgorithm");
    ros2_collision_detection::parameter_map_t param_map;
    try {
        // Load the plugin (circle algorithm in this case)
        std::shared_ptr<ros2_collision_detection::TTCAlgorithm> circle_algorithm = poly_loader.createSharedInstance("ros2_collision_detection_plugins::CircleAlgorithm");
        circle_algorithm->initialize(param_map);
        //circle_algorithm->initialize(10.0);

        std::shared_ptr<ros2_collision_detection::TTCAlgorithm> n_circle_algorithm = poly_loader.createSharedInstance("ros2_collision_detection_plugins::NCircleAlgorithm");
        n_circle_algorithm->initialize(param_map);

        //double accel_diff_sq_sin_adj = 2.0;  // Example value
        //double accel_diff_sq_cos_adj = 4.0;  // Example value

        //double result = circle_algorithm->computeCoefficientForPowerFour(accel_diff_sq_sin_adj, accel_diff_sq_cos_adj);
        //double result_equation = circle_equation_solver->computeCoefficientForPowerFour(accel_diff_sq_sin_adj, accel_diff_sq_cos_adj);

        // Create and populate the parameter_map_t
        //ros2_collision_detection::parameter_map_t parameter_map;
        //parameter_map["side_length"] = 10;  // Example: Side length for CircleAlgorithm

        // Pass the populated map to init
        //plugin->init(parameter_map);

        //RCLCPP_INFO(node->get_logger(), "Circle area: %.2f", circle_algorithm->area());
        //RCLCPP_INFO(node->get_logger(), "Circle Equation Solver area: %.2f", circle_equation_solver->area());
        //RCLCPP_INFO(node->get_logger(), "Result of computeCoefficientForPowerFour: %.2f", result);
        //RCLCPP_INFO(node->get_logger(), "Result of equation computeCoefficientForPowerFour: %.2f", result_equation);
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

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
    
}
*/