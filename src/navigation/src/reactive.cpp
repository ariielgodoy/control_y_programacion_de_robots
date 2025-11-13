#include "navigation/reactive.hpp"

using std::placeholders::_1;

reactive::reactive(): Node("reactive_navigation")
{
    pub_movement = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
    sub_laser = this -> create_subscription<sensor_msgs::msg::LaserScan>("/laser_scan", 10,
    std::bind(&reactive::execute_action, this, _1));
}

void reactive::execute_action(const sensor_msgs::msg::LaserScan::SharedPtr laser)const{
    
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<reactive>();

    rclcpp::Rate loop_rate(5.0);
    RCLCPP_INFO(node->get_logger(), "Starting Main Loop...");
    while(rclcpp::ok())
    {
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}