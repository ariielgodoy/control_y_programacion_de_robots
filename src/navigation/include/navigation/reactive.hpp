#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "navigation/srv/power.hpp"

class reactive: public rclcpp::Node
{
public:
    reactive();
    ~reactive();
    rclcpp::Service<navigation::srv::Power>::SharedPtr server_start_;

    bool active;
    
    void handle_start_service(const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<navigation::srv::Power::Request> request,
        std::shared_ptr<navigation::srv::Power::Response> response);

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_movement;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_laser;
    void spot_obstacle(const sensor_msgs::msg::LaserScan::SharedPtr laser)const;
    void execute_movement(bool obstacle_front, bool obstacle_left, bool obstacle_right, double front_distance, bool search_wall)const;

};