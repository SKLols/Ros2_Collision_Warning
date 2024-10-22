//
// Created by delooz on 05.03.20.
// Adapted from Silas Lobo work
// Migration to ROS2 from Abhishek Jagtap - July 19 2024

#include "cpm_provider.hpp"

#include "rclcpp/rclcpp.hpp"
//#include "std_msgs/String.h"

#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"
#include "v2xvf_interfaces/msg/map_object.hpp"
#include "ros_etsi_its_msgs/msg/perceived_object.hpp"
#include "ros_etsi_its_msgs/msg/list_of_perceived_objects.hpp"
#include "ros_etsi_its_msgs/msg/station_type.hpp"
#include "sensor_msgs/msg/magnetic_field.h"
#include "geometry_msgs/msg/vector3.h"
#include <stdio.h>
#include <string>
#include <iostream>
#include <math.h>
#include <functional>
#include <memory>
using std::placeholders::_1;




# define M_PIl          3.141592653589793238462643383279502884L /* pi */

using namespace std;

CPMReceiver::CPMReceiver(const rclcpp::NodeOptions &options)
    : Node("cpm_receiver_node", options)
{
    //m_publisher = this->create_publisher<v2xvf_interfaces::msg::MapObject>("map_manager", 10);
    m_sub_cpm = this->create_subscription<ros_etsi_its_msgs::msg::CPM>(
        "cpm_received", 10,
        [this](const ros_etsi_its_msgs::msg::CPM::SharedPtr cpm) {
            this->processCPM(cpm);
        });
}


void CPMReceiver::processCPM(const ros_etsi_its_msgs::msg::CPM::SharedPtr& cpm)
//void CPMReceiver::processCPM(const std::shared_ptr<ros_etsi_its_msgs::msg::CPM>& cpm)


{
    std::vector<ros_etsi_its_msgs::msg::PerceivedObject> vectorObject;
    vectorObject = cpm->list_of_perceived_objects.perceived_object_container;
    ros_etsi_its_msgs::msg::PerceivedObject objectDetected;
    std::vector<ros_etsi_its_msgs::msg::PerceivedObject>::iterator obj;

    //Match GPS point to Mobileye reference point
    double refPointLat = cpm->reference_position.latitude / 10000000.0; //Convert micro to degree
    double refPointLon = cpm->reference_position.longitude / 10000000.0; //Convert micro to degree
    double vehicleHeadingDegree = cpm->originating_vehicle_container.heading.value / 10.0; //From decidegree to degree
    double vehicleHeadingRad = vehicleHeadingDegree*M_PIl/180; //convert all values to RADIANS
    
    generateMapMarker(refPointLat, refPointLon, vehicleHeadingDegree, cpm->station_type.value, "CPM-S", cpm->its_header.station_id, cpm->its_header.station_id);

    refPointLat = refPointLat * M_PIl/180; //Convert degree to radian
    refPointLon = refPointLon * M_PIl/180; //Convert degree to radian


    for(obj = vectorObject.begin() ; obj != vectorObject.end() ; obj++) {

        objectDetected.object_id = obj->object_id;
        objectDetected.x_distance.value = obj->x_distance.value;
        objectDetected.y_distance.value = obj->y_distance.value;


        /* Important note: there are two different systems going on: heading is relatively to the north
         * Object angle is relatively to the vehicle, need to perform a conversion
         * */
        objectDetected.object_angle = atan2(objectDetected.y_distance.value, objectDetected.x_distance.value) * 180 / M_PIl;
        //std::cout << "station " << cpm->its_header.station_id <<  " x " << objectDetected.xDistance.value << "  y: " <<   obj->yDistance.value << " angle " << objectDetected.objectAngle << " heading " << vehicleHeadingDegree <<  std::endl;


        objectDetected.classification.value = obj->classification.value;

        /* Important note: conversion happens here! (take -)
            * */
        double objectAziDegree = vehicleHeadingDegree - objectDetected.object_angle;

        //Convert Degree to Rad
        double objectVehicleAngleRad =
                objectDetected.object_angle * M_PIl / 180; //angle between vehicle and object in Rad

        //Calculate the distance between object and vehicle
        double objectEuclideanDistance = sqrt(pow(objectDetected.x_distance.value, 2) + pow(objectDetected.y_distance.value, 2));//  / objectDetected.xDistance.value * abs(cos(objectVehicleAngleRad));

        //Calculate Object lat and long
        double objectLatRad;
        double objectLonRad;
        double objectLatDegree;
        double objectLonDegree;
        double objectAziRad;

        if (objectAziDegree < 0) {
            objectAziRad = (360 + objectAziDegree) * M_PIl / 180;
        }

        if (objectAziDegree > 360) {
            objectAziRad = (objectAziDegree - 360) * M_PIl / 180;
        } else {
            objectAziRad = objectAziDegree * M_PIl / 180;
        }

        //correct the surface inclination
        double objectGeographicalDistance = objectEuclideanDistance * abs(cos(0)); //Take into consideration pitch angle if available.

        //getting Object Lat and Long
        objectLatRad = asin(sin(refPointLat) * cos(objectGeographicalDistance / m_radiusEarth) +
                            cos(refPointLat) * sin(objectGeographicalDistance / m_radiusEarth) *
                            cos(objectAziRad));
        objectLonRad = refPointLon + atan2(sin(objectAziRad) * sin(objectGeographicalDistance / m_radiusEarth) *
                                           cos(refPointLat), cos(objectGeographicalDistance / m_radiusEarth) -
                                                             sin(refPointLat) * sin(objectLatRad));

        objectLatDegree = objectLatRad * (180 / M_PIl);
        objectLonDegree = objectLonRad * (180 / M_PIl);

        //Measure the error
        double aDistance;
        double cDistance;
        double dDistance;
        double diffe;

        aDistance = sin((objectLatRad - refPointLat) / 2) * sin((objectLatRad - refPointLat) / 2) +
                    cos(refPointLat) * cos(objectLatRad) * sin((objectLonRad - refPointLon) / 2) *
                    sin((objectLonRad - refPointLon) / 2);
        cDistance = 2 * atan2(sqrt(aDistance), sqrt(1 - aDistance));
        dDistance = m_radiusEarth * cDistance;
        diffe = abs(dDistance - objectGeographicalDistance);

        generateMapMarker(objectLatDegree, objectLonDegree, vehicleHeadingDegree, objectDetected.classification.value, "CPM-O", cpm->its_header.station_id, objectDetected.object_id);
    }
}


void CPMReceiver::generateMapMarker(double objectLatDegree, double objectLonDegree, double vehicleHeadingDegree,
        uint8_t type, std::string source, uint32_t stationId, uint32_t objectId){
    //Publishing
    v2xvf_interfaces::msg::MapObject marker;

    //rclpp::Time timeNow = rclpp::Time::now();
    //rclcpp::Time time_now = get_clock()->now();
    //Node->get
    //marker.header.stamp = timeNow;
    //marker.header.timestamp = time_now
    //marker.header.frame_id = "cpm";
    marker.position.latitude = objectLatDegree * 10000000;
    marker.position.longitude = objectLonDegree * 10000000;
    marker.heading.value = vehicleHeadingDegree*10;

    //define object type
    switch (type){
        case ros_etsi_its_msgs::msg::StationType::PASSENGER_CAR:
            marker.type = "VEHICLE";
            break ;
        case ros_etsi_its_msgs::msg::StationType::LIGHT_TRUCK:
            marker.type = "TRUCK";
            break;
        case ros_etsi_its_msgs::msg::StationType::HEAVY_TRUCK:
            marker.type = "TRUCK";
            break;
        case ros_etsi_its_msgs::msg::StationType::MOTORCYCLE:
            marker.type = "MOTORCYCLE";
            break;
        case ros_etsi_its_msgs::msg::StationType::PEDESTRIAN:
            marker.type = "PEDESTRIAN";
            break;
        case ros_etsi_its_msgs::msg::StationType::CYCLIST:
            marker.type = "CYCLIST";
            break;
        case ros_etsi_its_msgs::msg::StationType::ROAD_SIDE_UNIT:
            marker.type = "RSU";
            break;
        case ros_etsi_its_msgs::msg::StationType::MOPED:
            marker.type = "PDK";
            break;
        case ros_etsi_its_msgs::msg::StationType::SPECIAL_VEHICLE:
            marker.type = "Level0";
            break;
        case ros_etsi_its_msgs::msg::StationType::TRAM:
            marker.type = "Level1";
            break;
        case ros_etsi_its_msgs::msg::StationType::TRAILER:
            marker.type = "Level2";
            break;
        case ros_etsi_its_msgs::msg::StationType::BUS:
            marker.type = "Level3";
            break;            
        default:
            marker.type = "UNKNOWN";
    }

    marker.source = source;
    marker.source_id = stationId;

    marker.expiration_time = 1.1;

    if(m_cpms_received.count(stationId + objectId)){
        marker.id = m_cpms_received[stationId + objectId];
    } else {
        boost::uuids::uuid uuid = boost::uuids::random_generator()();
        marker.id = boost::uuids::to_string(uuid);
        m_cpms_received[stationId + objectId] = marker.id;
    }

    //m_publisher.publish(marker);
    m_publisher->publish(marker);

        //RCLCPP_INFO(this->get_logger(), "Publishing message: '%s'", message.field_name.c_str());

}