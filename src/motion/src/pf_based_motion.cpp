#include <motion/pf_based_motion.hpp>
using std::placeholders::_1;

MM::MM(): Node("motion_model")
{
    sub_pose_ = this->create_subscription<geometry_msgs::msg::Pose>("/estimated_pose", 1, std::bind(&MM::ModifyMovement, this, _1));
    pub_twist_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
}

MM::~MM()
{

}

void MM::ModifyMovement(const geometry_msgs::msg::Pose::SharedPtr msg)const{
    geometry_msgs::msg::Twist movement;
    float linear, angular;
    float x, y, z;
    float x_objetivo = 5;
    float y_objetivo = 5;
    float z_objetivo = 0;
    float robot_angle;
    float euclidean_distance;
    x = msg->position.x;
    y = msg->position.y;
    z = msg->orientation.z;
    euclidean_distance = sqrt((x_objetivo-x)*(x_objetivo-x)+(y_objetivo-y)*(y_objetivo-y));

    robot_angle = atan2(y_objetivo-y, x_objetivo-x);

    if (z > robot_angle + M_PI/10){
        angular = -M_PI/8;
    }else if (z < robot_angle - M_PI/10){
        angular = M_PI/8;
    }else{
        angular = 0;
    }
    if(euclidean_distance < 0.8){
        linear = 0;
        if(z > z_objetivo + M_PI/10 ){
            angular = -M_PI/8;
        }else if(z<z_objetivo - M_PI/10){
            angular = M_PI/8;
        }
        else{
            angular = 0;
        }
    }else{
        linear = 0.1;
    }
    movement.linear.x = linear;
    movement.angular.z = angular;
    
    this->pub_twist_->publish(movement);
    
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MM>();
    RCLCPP_INFO(node->get_logger(), "Starting Main Loop...");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}