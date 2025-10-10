#include <motion/motion_model.hpp>

MM::MM(): Node("motion_model")
{
    pub_pose_ = this->create_publisher<geometry_msgs::msg::Pose>("/estimated_pose", 1);
    pub_twist_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);

    estimatedPose.position.x = 0;
    estimatedPose.position.y = 0;
    estimatedPose.orientation.z = 0;
}


geometry_msgs::msg::Pose MM::pose_composition(geometry_msgs::msg::Pose p1, geometry_msgs::msg::Pose p2)
{
    _Float64 x1 = p1.position.x;
    _Float64 y1 = p1.position.y;
    _Float64 theta1 = p1.orientation.z;
    _Float64 x2 = p2.position.x;
    _Float64 y2 = p2.position.y;
    _Float64 theta2 = p2.orientation.z;

    geometry_msgs::msg::Pose p;
    p.position.x = x1 + x2*cos(theta1) - y2*sin(theta1);
    p.position.y = y1 + x2*sin(theta1) + y2*cos(theta1);
    p.orientation.z = theta1 + theta2;
    return p;
}

void MM::step_forward()
{
    double Vp = 0;
    double w = 0;
    double At = 0.2;
    geometry_msgs::msg::Pose pAt;
    geometry_msgs::msg::Twist movement;
    geometry_msgs::msg::Pose new_position;

    if (counter < 75)
    {
        Vp = 0.2;
        w = 0;
        pAt.position.x = (Vp*At);
        pAt.position.y = 0;
        pAt.orientation.z = 0;
    }
    else if (counter < 95){
        Vp = 0;
        w = -M_PI/8;
        pAt.position.x = (Vp/w) * sin(w*At);
        pAt.position.y = (Vp/w) * (1-cos(w*At));
        pAt.orientation.z = w*At;
    }
    else{
        Vp = 0;
        w = 0;
        counter = 0;
    }
    counter++;

    new_position = this->pose_composition(this->estimatedPose, pAt);
    this->estimatedPose = new_position;
    movement.linear.x = Vp;
    movement.angular.z = w;

    RCLCPP_INFO(this->get_logger(), "Counter=%d | x=%.2f, y=%.2f, z=%.2f",
                counter,
                estimatedPose.position.x,
                estimatedPose.position.y,
                estimatedPose.orientation.z);

    pub_pose_->publish(new_position);
    pub_twist_->publish(movement);
}


MM::~MM()
{

}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MM>();

    rclcpp::Rate loop_rate(5.0);
    RCLCPP_INFO(node->get_logger(), "Starting Main Loop...");
    while(rclcpp::ok())
    {
        rclcpp::spin_some(node);
        node->step_forward();
        loop_rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}