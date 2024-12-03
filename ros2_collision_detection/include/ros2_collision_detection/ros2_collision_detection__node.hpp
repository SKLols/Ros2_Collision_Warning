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
 * @date 2024-11-28
 * @brief Updated from ROS 1 to ROS 2.
 * 
 */

#ifndef _COLLISION_DETECTION_H_
#define _COLLISION_DETECTION_H_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <pluginlib/class_loader.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <v2xvf_interfaces/msg/collision_check_result.hpp>
#include <v2xvf_interfaces/msg/object_movement.hpp>
#include <v2xvf_interfaces/msg/perceived_object_motion.hpp>
#include <v2xvf_interfaces/msg/perceived_objects.hpp>
#include <v2xvf_interfaces/msg/subject_vehicle_motion.hpp>

#include <ros2_collision_detection/ttc_algorithm.hpp>
//@todo below 3 files
#include <ros2_collision_detection/ttc_calculator.hpp>
#include <ros2_collision_detection/warning_generator.hpp>
#include <ros2_collision_detection/ttc_only_warning_algorithm.hpp>

/**
 * @brief Typedef for a approximate sync policy that synchronizes a PerceivedObjects message with a SubjectVehicleMotion message.
 * 
 */
//typedef message_filters::sync_policies::ApproximateTime<ros_collision_detection::PerceivedObjects, ros_collision_detection::SubjectVehicleMotion> ApproximateSyncPolicy;
using ApproximateSyncPolicy = message_filters::sync_policies::ApproximateTime<v2xvf_interfaces::msg::PerceivedObjects, v2xvf_interfaces::msg::SubjectVehicleMotion>;

/**
 * @brief Main class that manages ROS publishers and subscribers and initializes the calculation and warning components.
 * 
 */

class CollisionDetection
{
    private:
        /**
         * @brief The ROS node handle for the node.
         * 
         */
        //ros::NodeHandle *node_handle;
        std::shared_ptr<rclcpp::Node> node_handle;

        /**
         * @brief Class loader that can load classes that implement interface TTCAlgorithm.
         * 
         */
        pluginlib::ClassLoader<ros2_collision_detection::TTCAlgorithm> ttc_algorithm_loader;

        /**
         * @brief Class loader that can load classes that implement interface WarningGeneratorAlgorithm.
         * 
         */
        
        pluginlib::ClassLoader<WarningGeneratorAlgorithm> warning_generator_algorithm_loader;

        /**
         * @brief ROS message filters subscriber to topic "/fused_objects".
         * 
         */
        message_filters::Subscriber<v2xvf_interfaces::msg::PerceivedObjects> fused_objects_subscriber;

        /**
         * @brief ROS message filters subscriber to topic "ego_position".
         * 
         */
        message_filters::Subscriber<v2xvf_interfaces::msg::SubjectVehicleMotion> ego_position_subscriber;

        /**
         * @brief ROS message filters synchronizer that uses approximate sync policy.
         * 
         */
        //message_filters::Synchronizer<ApproximateSyncPolicy> approximate_synchronizer;

        /**
        * @brief ROS publisher for collision warnings to topic "/collision_warning".
        * 
        */
        rclcpp::Publisher<v2xvf_interfaces::msg::CollisionCheckResult>::SharedPtr collision_warning_publisher;

        /**
         * @brief Object responsible for calculating Time-To-Collision from received ROS messages.
         * 
         */
        
        TTCCalculator ttc_calculator;

        /**
         * @brief Object responsible for generating and sending collision warnings.
         * 
         */
        
        WarningGenerator warning_generator;

        /**
         * @brief Initialize the subscribers, the synchronizer and the publisher.
         * 
         */
        void init();

        /**
         * @brief Initialize the ROS node from the launch parameters.
         * 
         */
        void initFromLaunchParameters();

        /**
         * @brief Check the launch parameters and set defaults if necessary.
         * 
         */
        void checkLaunchParameters();

        /**
         * @brief Load the necessary plugins for the TTC calculator.
         * 
         */
        void loadPlugins();

        /**
         * @brief Initialize the node components with the launch parameters.
         * 
         */
        void initComponents();
    public:
    /**
     * @brief Construct a new Collision Detection object.
     * 
     * Construct a new Collision Detection object with the passed node handle. 
     * The new Collision Detection object will be initialized and the TTC Calculator and 
     * the Warning Generator are initialized with concrete calculation and generation strategies.
     * 
     * @param nh Pointer to the ROS node handle.
     */
    //CollisionDetection(ros::NodeHandle *nh);
    CollisionDetection(std::shared_ptr<rclcpp::Node> nh);

    /**
     * @brief Call TTC Calculator to calculate the TTC from the two passed messages.
     * 
     * A Callback that is invoked when the subscribers both received a message at approximately the same time.
     * The TTC Calculator then is called to calculate the TTC from the two passed messages.
     * 
     * @param perceived_objects_msg The message containing the motions of the perceived objects.
     * @param subject_vehicle_motion_msg The message containing the motion of the subject vehicle.
     */
    //void callback(const ros_collision_detection::PerceivedObjectsConstPtr& perceived_objects_msg, const ros_collision_detection::SubjectVehicleMotionConstPtr& subject_vehicle_motion_msg);
    void callback(
        const v2xvf_interfaces::msg::PerceivedObjects::SharedPtr perceived_objects_msg,
        const v2xvf_interfaces::msg::SubjectVehicleMotion::SharedPtr subject_vehicle_motion_msg
    );


};

#endif  // _COLLISION_DETECTION_H_