#include <motion/motion_model_noise.hpp>
#include <random>
#include <motion/json.hpp>
using json = nlohmann::json;

MM::MM(): Node("motion_model_noise")
{
    //pub_pose_ = this->create_publisher<geometry_msgs::msg::Pose>("/estimated_pose", 1);

    pub_pose_ = this->create_publisher<std_msgs::msg::String>("/estimated_pose_array", 1);
    pub_twist_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);

    estimatedPose.position.x = 0;
    estimatedPose.position.y = 0;
    estimatedPose.orientation.z = 0;
    
    estimated_pose_array.resize(100);

    for(int j = 0; j < 100; j++){
        estimated_pose_array[j] = estimatedPose;
    }
}

geometry_msgs::msg::Pose MM::generate_noise_sample()
{
    // Seed with a real random value, if available
    std::random_device r;
    std::seed_seq seed2{r(), r(), r(), r(), r(), r(), r(), r()};
    std::mt19937 generator(seed2);
    // Noise Covariance_Matriz (only diagonal) [var_x, var_y, var_theta]
    std::vector<double> var_noise = {0.001, 0.001, 0.001};
    // normal_distribution(mean, std_dev)
    std::normal_distribution<> norm_dist_x(0,sqrt(var_noise[0]));
    std::normal_distribution<> norm_dist_y(0,sqrt(var_noise[1]));
    std::normal_distribution<> norm_dist_theta(0,sqrt(var_noise[2]));
    // Create a noise sample and store in a Pose variable
    geometry_msgs::msg::Pose noise_sample;
    noise_sample.position.x = norm_dist_x(generator);
    noise_sample.position.y = norm_dist_y(generator);
    noise_sample.orientation.z = norm_dist_theta(generator);
    return noise_sample;
}


std::string MM::poses_to_json(std::vector<geometry_msgs::msg::Pose> vec_poses)
{
    json json_vector_poses; //creamos objeto tipo JSON (resultado)
    // Por cada pose de nuestro vector
    for (geometry_msgs::msg::Pose value : vec_poses)
    {
        // Transformamos a JSON
        json json_pose;
        json_pose["x"] = value.position.x;
        json_pose["y"] = value.position.y;
        json_pose["phi"] = value.orientation.z;
        // attach JSON value
        json_vector_poses.push_back(json_pose);
    }
    // return
    return json_vector_poses.dump(); // Esto es una cadena de texto!
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
    geometry_msgs::msg::Pose noise_sample;
    geometry_msgs::msg::Pose pAt;
    geometry_msgs::msg::Pose p_inc_ruidosa;
    geometry_msgs::msg::Twist movement;
    geometry_msgs::msg::Pose new_position;
    std_msgs::msg::String serialized_pose_array;

    

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

    for(int i = 0; i < 100; i++){
        noise_sample = generate_noise_sample();
        p_inc_ruidosa = this->pose_composition(pAt, noise_sample);
        new_position = this->pose_composition(this->estimated_pose_array[i], p_inc_ruidosa);
        this->estimated_pose_array[i] = new_position;
        
        //this->estimatedPose = new_position;
    }

    serialized_pose_array.data = this->poses_to_json(estimated_pose_array);
    movement.linear.x = Vp;
    movement.angular.z = w;

    /*RCLCPP_INFO(this->get_logger(), "Counter=%d | x=%.2f, y=%.2f, z=%.2f",
                counter,
                estimatedPose.position.x,
                estimatedPose.position.y,
                estimatedPose.orientation.z);*/

    pub_pose_->publish(serialized_pose_array);
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