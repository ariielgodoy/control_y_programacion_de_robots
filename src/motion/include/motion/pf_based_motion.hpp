#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"

class MM : public rclcpp::Node
{
public:
    MM();
    ~MM();
    void ModifyMovement(const geometry_msgs::msg::Pose::SharedPtr msg)const;
    
private:
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_pose_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_twist_;
};