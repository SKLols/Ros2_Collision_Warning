#include "cpm_provider.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    auto node_options = rclcpp::NodeOptions();
    auto node = std::make_shared<CPMReceiver>(node_options);

    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}
