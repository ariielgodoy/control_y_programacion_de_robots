#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/string.hpp"

class MM : public rclcpp::Node
{
public:
    MM();
    ~MM();

    void step_forward();

private:
    int counter = 0;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_pose_;
    //rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_pose_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_twist_;

    geometry_msgs::msg::Pose estimatedPose;
    std::vector<geometry_msgs::msg::Pose> estimated_pose_array;
    geometry_msgs::msg::Pose pose_composition(geometry_msgs::msg::Pose p1, geometry_msgs::msg::Pose p2);
    geometry_msgs::msg::Pose generate_noise_sample();
    std::string poses_to_json(std::vector<geometry_msgs::msg::Pose> vec_poses);
};