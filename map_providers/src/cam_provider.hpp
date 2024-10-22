//
// Created by rclcppk on 30.08.19.
//

#include "rclcpp/rclcpp.hpp"
#include <algorithm>
#include <iostream>
#include <cstdint>
#include <cstring>
#include <random>
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"
#include "v2xvf_interfaces/msg/map_object.hpp"
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <ros_etsi_its_msgs/msg/cam.hpp>

class CamReceiver
{
    public:
        CamReceiver(rclcpp::Node::SharedPtr nh, rclcpp::Node::SharedPtr nh_private, rclcpp::Publisher<ros_etsi_its_msgs::msg::CAM>::SharedPtr publisher);
        
        //: Node("cam_receiver"), m_nh(nh), m_nh_private(nh_private), m_publisher(publisher)
        void receiveCam(const ros_etsi_its_msgs::msg::CAM::SharedPtr& cam);

    private:
        rclcpp::Node::SharedPtr m_nh;
        rclcpp::Node::SharedPtr m_nh_private;
        rclcpp::Subscription<ros_etsi_its_msgs::msg::CAM>::SharedPtr m_sub;
        rclcpp::Publisher<ros_etsi_its_msgs::msg::CAM>::SharedPtr m_publisher;
        std::map<uint32_t, std::string> m_cams_received;
};