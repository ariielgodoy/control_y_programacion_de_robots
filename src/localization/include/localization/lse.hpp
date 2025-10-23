#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <eigen3/Eigen/Dense> 
#include "localization/srv/set_estimation.hpp"

#include <string>
#define MAX_IT 1000
#define eps 0.0000001

class Lse : public rclcpp::Node
{
public:
    Lse();
    ~Lse();

private:
    // Publishers & subscribers
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
    rclcpp::Service<localization::srv::SetEstimation>::SharedPtr server_;

    // Callbacks & Funcitions
    void get_parameters();
    void execute_lse_only_range(const std_msgs::msg::String::SharedPtr msg);
    void handle_set_estimation(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<localization::srv::SetEstimation::Request> request,
        const std::shared_ptr<localization::srv::SetEstimation::Response> response
    );

    // Aux functions
    double angleWrap(double ang);
    double angleDiff(double ang1, double ang2);
    
    // Robot Position estimation
    Eigen::Vector3d est;
    geometry_msgs::msg::Pose estimatedPose;
        
    // vars & params
    int num_beacons;
    std::vector<Eigen::Vector2d> beacons_pos;
    std::vector<std::string> beacons_names;
    std::vector<double> initial_est;    
    bool use_var;
    Eigen::VectorXd var_range;
    bool use_var_range_proportional;
};
