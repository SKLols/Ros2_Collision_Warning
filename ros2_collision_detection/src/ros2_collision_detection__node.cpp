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


// definition of default values for launch parameters_
#define DEFAULT_PUBLISH_TOPIC "/collision_warning"
#define DEFAULT_TTC_ALGORITHM_CLASSNAME "ros2_collision_detection_plugins::CircleAlgorithm"
#define DEFAULT_WARNING_GENERATOR_ALGORITHM_CLASSNAME "TTCOnlyWarningAlgorithm"
#define DEFAULT_TTC_ALGORITHM_CIRCLE_COUNT 1
#define DEFAULT_SUBJECT_VEHICLE_LENGTH 5
#define DEFAULT_SUBJECT_VEHICLE_WIDTH 1.8


CollisionDetection::CollisionDetection(std::shared_ptr<rclcpp::Node> nh)
: Node("Collision_Detection"),
ttc_algorithm_loader("ros2_collision_detection", "ros2_collision_detection::TTCAlgorithm"),
warning_generator_algorithm_loader("ros2_collision_detection", "WarningGeneratorAlgorithm"),
approximate_synchronizer(ApproximateSyncPolicy(10), fused_objects_subscriber, ego_position_subscriber),
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
    std::function<void(std::shared_ptr<const v2xvf_interfaces::msg::SubjectVehicleMotion>, 
                       std::shared_ptr<const v2xvf_interfaces::msg::PerceivedObjectMotion>, 
                       double)>
        bound_function = std::bind(&WarningGenerator::createWarning, &warning_generator, 
                                   std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);

    // Pass the bound function to the callback
    ttc_calculator.addWarningSignalCallback(bound_function);   
    
    RCLCPP_INFO(node_handle->get_logger(), "Node successfully initialized-4.");
    //@todo
    //collision_warning_publisher = node_handle->create_publisher<v2xvf_interfaces::msg::CollisionCheckResult>("/collision_warning", 10);
    fused_objects_subscriber.subscribe(node_handle, "/fused_objects");
    ego_position_subscriber.subscribe(node_handle, "/ego_position");
    approximate_synchronizer.registerCallback(std::bind(&CollisionDetection::callback, this, std::placeholders::_1, std::placeholders::_2));
    
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

void CollisionDetection::callback(const v2xvf_interfaces::msg::PerceivedObjects::ConstSharedPtr& perceived_objects_msg,const v2xvf_interfaces::msg::SubjectVehicleMotion::ConstSharedPtr& subject_vehicle_motion_msg)
{
    // log seq number of the two messages
    RCLCPP_DEBUG(node_handle->get_logger(),"CollisionDetection::callback: Subject vehicle msg: seq = %d | perceived object msg: seq = %d.", subject_vehicle_motion_msg->header.stamp.sec, subject_vehicle_motion_msg->header.stamp.nanosec, perceived_objects_msg->header.stamp.sec, perceived_objects_msg->header.stamp.nanosec);
    
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
