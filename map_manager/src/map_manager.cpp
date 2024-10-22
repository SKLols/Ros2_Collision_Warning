#include "rclcpp/rclcpp.hpp"
#include "v2xvf_interfaces/msg/map_object.hpp"
#include "v2xvf_interfaces/msg/list_map_objects.hpp"
#include "memory"
#include "algorithm"
#include "iostream"
#include "cstdint"
#include "cstring"
#include "random"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"
#include "boost/uuid/uuid.hpp"
#include "boost/uuid/uuid_generators.hpp"
#include "boost/uuid/uuid_io.hpp"
#include "rclcpp/logging.hpp"

std::map<std::string, v2xvf_interfaces::msg::MapObject> map_objects;

void receiveObjects(const v2xvf_interfaces::msg::MapObject::SharedPtr msg)
{
    map_objects[msg->id] = *msg; // In case we have more than one update for the same object
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("map_manager_node");
    node->declare_parameter<int>("update_frequency", 5);
    //int rate;

    int rate = node->get_parameter("update_frequency").as_int();


    //if (!node->get_parameter("args_updating_frequency", rate)) {
      //  RCLCPP_ERROR(node->get_logger(), "Failed to get parameter 'updating_frequency'");
        //return 1;
    //}
    rclcpp::Rate r(rate);
    //rclcpp::Rate r(5);

    auto publisher_map_objects = node->create_publisher<v2xvf_interfaces::msg::ListMapObjects>("map_objects", 1000);
    auto sub = node->create_subscription<v2xvf_interfaces::msg::MapObject>("map_manager", 1000, receiveObjects);

    v2xvf_interfaces::msg::ListMapObjects list_map_object;
    std::vector<v2xvf_interfaces::msg::MapObject> list_objects;
    v2xvf_interfaces::msg::MapObject mapObject;

    mapObject.position.latitude = 48.765957 * 1000000; // In micro degree
    mapObject.position.longitude = 11.434670 * 1000000;

    mapObject.heading.value = 2300;
    mapObject.type = "VEHICLE";
    mapObject.source = "GPS";

    boost::uuids::uuid uuid = boost::uuids::random_generator()();
    mapObject.id = boost::uuids::to_string(uuid);

    mapObject.expiration_time = 1; // in second

    list_objects.push_back(mapObject);

    list_map_object.list_objects = list_objects;

    while (rclcpp::ok()) {

        publisher_map_objects->publish(list_map_object);
        rclcpp::spin_some(node);

        if (!map_objects.empty()) {

            for (auto obj : map_objects)
                list_objects.push_back(obj.second);

            list_map_object.list_objects = list_objects;

            publisher_map_objects->publish(list_map_object);

            // No need to keep the objects already transmitted once
            map_objects.clear();
            list_objects.clear();
        }

        r.sleep();
    }
}
