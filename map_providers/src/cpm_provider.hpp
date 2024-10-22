#ifndef MAP_CPM_PROVIDER_H
#define MAP_CPM_PROVIDER_H

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <ros_etsi_its_msgs/msg/perceived_object.hpp>
#include <ros_etsi_its_msgs/msg/list_of_perceived_objects.hpp>
#include <ros_etsi_its_msgs/msg/cpm.hpp>
#include "v2xvf_interfaces/msg/map_object.hpp"
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <string>
#include <map>
#include <cmath>

class CPMReceiver : public rclcpp::Node
{
public:
    CPMReceiver(const rclcpp::NodeOptions &options);
    void processCPM(const ros_etsi_its_msgs::msg::CPM::SharedPtr& cpm);
    void generateMapMarker(double objectLatDegree, double objectLonDegree, double vehicleHeadingDegree,
                           uint8_t type, std::string source, uint32_t stationId, uint32_t objectId);

private:
    rclcpp::Publisher<v2xvf_interfaces::msg::MapObject>::SharedPtr m_publisher;
    rclcpp::Subscription<ros_etsi_its_msgs::msg::CPM>::SharedPtr m_sub_cpm;
    std::string m_id;
    std::map<uint32_t, std::string> m_cpms_received;
    const double m_radiusEarth = 6371000;
};

#endif // MAP_CPM_PROVIDER_H
