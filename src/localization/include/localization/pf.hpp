#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
// Other libs
#include <eigen3/Eigen/Dense>
#include <pcl_conversions/pcl_conversions.h>
#include <random>
#include <string>
// For SRV
#include "localization/srv/set_estimation.hpp"


class Pf : public rclcpp::Node
{
public:
    Pf();
    void update_pf(const geometry_msgs::msg::Twist::SharedPtr msg);
    void execute_pf(const std_msgs::msg::String::SharedPtr msg);
    void get_parameters();
    ~Pf();

private:
    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_;                // Estimated Pose
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_;     // PointCloud
    
    // Subscribers
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;           // Measurements
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_twist_;      // cmd_vel
    
    rclcpp::Service<localization::srv::SetEstimation>::SharedPtr server_;
    void handle_set_estimation(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<localization::srv::SetEstimation::Request> request,
        const std::shared_ptr<localization::srv::SetEstimation::Response> response
    );

    //Particles 
    unsigned num_particles;
    Eigen::MatrixXd p;

    //current pose estimation
    Eigen::Vector3d est;
    double sparse;      //dimmensions of the square where particles are generated. It can be 0
    geometry_msgs::msg::Pose estimatedPose;
    
    //beacons positions <x,y>
    unsigned num_beacons;
    std::vector<std::string> beacons_names;
    std::vector<Eigen::Vector2d> beacons_pos;
    
    Eigen::Vector3d var_motion;  //motion variance for inc x,y,theta

    //variance of each beacon
    Eigen::VectorXd var_range;
    Eigen::VectorXd var_bearing;

    void tcomp(Eigen::Vector3d pose,Eigen::Vector3d inc,Eigen::Vector3d &res);
    double AngleWrap(double ang);
    double AngleDiff(double ang1, double ang2);
    void publishCloud();

    rclcpp::Time last;
    bool started;
    geometry_msgs::msg::Twist last_cmd_vel;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
};
