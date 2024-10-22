//
// Created by rosk on 30.08.19.
//
#include "rclcpp/rclcpp.hpp"
#include "algorithm"
#include "iostream"
#include "cstdint"
#include "cstring"
#include "random"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"
#include "v2xvf_interfaces/msg/map_object.hpp"
#include "boost/uuid/uuid.hpp"
#include "boost/uuid/uuid_generators.hpp"
#include "boost/uuid/uuid_io.hpp"
#include "ros_etsi_its_msgs/msg/cam.hpp"
#include "cam_provider.hpp"
#include "rclcpp/logging.hpp"


CamReceiver::CamReceiver(rclcpp::Node::SharedPtr nh, rclcpp::Node::SharedPtr nh_private, rclcpp::Publisher<ros_etsi_its_msgs::msg::CAM>::SharedPtr publisher);
    //m_nh(nh), m_nh_private(nh_private), m_publisher(publisher)
    {

    std::string cam_topic;
    //m_nh_private.getParam("cam_provider_topic", cam_topic);

    //m_sub = nh.subscribe("cam_received", 1000, &CamReceiver::receiveCam, this);
    m_sub = this->create_subscription<ros_etsi_its_msgs::msg::CAM>(
        "cam_received", 10, std::bind(&CamReceiver::receiveCam, this, std::placeholders::_1));
}


void CamReceiver::receiveCam(const ros_etsi_its_msgs::CAMPtr& cam) {
    v2xvf_interfaces::msg::MapObject mapObject;
    
    mapObject.position = cam->reference_position; //In micro degree * 10 (from standard)
    mapObject.heading = cam->high_frequency_container.heading;

    switch (cam->station_type.value){
        case ros_etsi_its_msgs::StationType::PASSENGER_CAR:
            mapObject.type = "VEHICLE";
            break ;
        case ros_etsi_its_msgs::StationType::LIGHT_TRUCK:
            mapObject.type = "TRUCK";
            break;
        case ros_etsi_its_msgs::StationType::HEAVY_TRUCK:
            mapObject.type = "TRUCK";
            break;
        case ros_etsi_its_msgs::StationType::MOTORCYCLE:
            mapObject.type = "MOTORCYCLE";
            break;
        case ros_etsi_its_msgs::StationType::PEDESTRIAN:
            mapObject.type = "PEDESTRIAN";
            break;
        case ros_etsi_its_msgs::StationType::CYCLIST:
            mapObject.type = "CYCLIST";
            break;
        case ros_etsi_its_msgs::StationType::ROAD_SIDE_UNIT:
            mapObject.type = "RSU";
            break;
        case ros_etsi_its_msgs::StationType::MOPED:
            mapObject.type = "PDK";
            break;
        case ros_etsi_its_msgs::StationType::SPECIAL_VEHICLE:
            mapObject.type = "Level0";
            break;
        case ros_etsi_its_msgs::StationType::TRAM:
            mapObject.type = "Level1";
            break;
        case ros_etsi_its_msgs::StationType::TRAILER:
            mapObject.type = "Level2";
            break;
        case ros_etsi_its_msgs::StationType::BUS:
            mapObject.type = "Level3";
            break;            
        default:
            mapObject.type = "UNKNOWN";
    }

    mapObject.source = "CAM";
    mapObject.source_id = cam->its_header.station_id;

    if(m_cams_received.count(cam->its_header.station_id)){
        mapObject.id = m_cams_received[cam->its_header.station_id];
    } else {
        boost::uuids::uuid uuid = boost::uuids::random_generator()();
        mapObject.id = boost::uuids::to_string(uuid);
        m_cams_received[cam->its_header.station_id] = mapObject.id;
    }

    mapObject.expiration_time = 1.3; //in second

    m_publisher.publish(mapObject);
}
