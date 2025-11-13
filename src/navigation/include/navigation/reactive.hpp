#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class reactive: public rclcpp::Node
{
public:
    reactive();
    ~reactive();

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_movement;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_laser;
    void execute_action(const sensor_msgs::msg::LaserScan::SharedPtr laser)const;
};