#include "navigation/reactive.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

reactive::reactive(): Node("reactive_navigation")
{
    server_start_ = this->create_service<navigation::srv::Start>
    ("start_service", std::bind(&reactive::handle_start_service, this,
    _1,_2,_3));
    pub_movement = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
    sub_laser = this -> create_subscription<sensor_msgs::msg::LaserScan>("/laser_scan", 10,
    std::bind(&reactive::spot_obstacle, this, _1));

    active = true;
}

reactive::~reactive(){}

void reactive::handle_start_service(const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<navigation::srv::Start::Request> request,
        std::shared_ptr<navigation::srv::Start::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "Service start_service Triggered");
    this->active = !this->active;
}


void reactive::execute_movement(bool obstacle_front, bool obstacle_left, bool obstacle_right, double front_distance, bool search_wall)const{
    geometry_msgs::msg::Twist movement;
    if(this->active){
        movement.linear.x = front_distance/5;
        movement.angular.z = 0;
        if (obstacle_front & obstacle_right){
            movement.linear.x = 0.2;
            movement.angular.z = 0.7;
        }else if(obstacle_front & obstacle_left){
            movement.linear.x = 0.2;
            movement.angular.z = -0.7;
        }else if(obstacle_front & !search_wall){
            movement.linear.x = front_distance/8;
            movement.angular.z = 0.7;
        }else if(obstacle_front){
            movement.linear.x = front_distance/8;
            movement.angular.z = -0.7;
        }else if(search_wall){
            movement.angular.z = -0.3;
        }
    }else{
        RCLCPP_INFO(this->get_logger(), "Movimiento detenido por servicio");
        movement.linear.x = 0.0;
        movement.angular.z = 0.0;

    }

    this->pub_movement->publish(movement);
}


void reactive::spot_obstacle(const sensor_msgs::msg::LaserScan::SharedPtr laser)const{
    int array_size = laser->ranges.size();
    //double angle_min = laser->angle_min; //= -120
    double angle_max = laser->angle_max; //= 120
    double angle_increment = laser->angle_increment;

    int positions_to_look_sideways = (angle_max-M_PI/2)/angle_increment;
    int left_laser = array_size - 1 - positions_to_look_sideways;
    int right_laser = positions_to_look_sideways;
    int front_laser = array_size/2;
    int sweep = 0.384/angle_increment; //~=22 grados pasados a
    //posiciones de array, ademas uso int para truncar el calculo

    //Parametros para los bucles
    double left_sweep_start = left_laser - sweep;
    double left_sweep_end = left_laser + sweep;

    double front_sweep_start = front_laser - sweep;
    double front_sweep_end = front_laser+ sweep;

    double right_sweep_start = right_laser - sweep;
    double right_sweep_end = right_laser + sweep;

    //Flags para el programa de movimiento
    bool obstacle_front = false;
    bool obstacle_left = false;
    bool obstacle_right = false;
    bool search_wall = true;
    double front_distance = laser->range_max;


    //Aqui busco los obstaculos por la izquierda
    for(int i = left_sweep_start; i < left_sweep_end; i++){
        if(laser->ranges[i] < 2){
            obstacle_left = true;
            //RCLCPP_INFO(this->get_logger(), "IZQUIERDA");
        }
    }

    //Aqui busco los obstaculos de delante
    for(int j = front_sweep_start; j < front_sweep_end; j++){
        if(laser->ranges[j] < 2){
            obstacle_front = true;
            if(front_distance > laser->ranges[j]){
                front_distance = laser->ranges[j];
            }
        }
    }

    //Aqui busco los obstaculos de la derecha
    for(int k = right_sweep_start; k < right_sweep_end; k++){
        if(laser->ranges[k] < 2){
            obstacle_right = true;
            //RCLCPP_INFO(this->get_logger(), "DERECHA");
        }
        if(laser->ranges[k]<laser->range_max-0.1){
            search_wall = false;
        }
    }

    this->execute_movement(obstacle_front, obstacle_left, obstacle_right, front_distance, search_wall);
        
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<reactive>();
    RCLCPP_INFO(node->get_logger(), "Initiating reactive motion...");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
