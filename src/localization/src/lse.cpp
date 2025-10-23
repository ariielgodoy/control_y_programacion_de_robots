#include <localization/lse.hpp>
#include <localization/json.hpp>

using json = nlohmann::json;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;


Lse::Lse(): Node ("localization_lse")
{
    // Read Node parameters
    get_parameters();

    // Publishers
    pub_ = this->create_publisher<geometry_msgs::msg::Pose> ( "/estimated_pose", 10 );

    // Subscribers    
    sub_ = this->create_subscription<std_msgs::msg::String>("/range_scan", 10, std::bind(&Lse::execute_lse_only_range, this, _1) );
    
    // Services
    server_ = this->create_service<localization::srv::SetEstimation>("set_estimation",std::bind(&Lse::handle_set_estimation,this,_1,_2,_3));
}


Lse::~Lse()
{
}


// Read Parameters (configuration)    
void Lse::get_parameters()
{
    // Declare parameters of this node (name, initial_value)
    this->declare_parameter("num_beacons",3);
    this->declare_parameter("beacons_pos",std::vector<double>{0.0,3.0,3.0,0.0,0.0,-3.0} );
    this->declare_parameter("beacons_names",std::vector<std::string>{"beacon_1","beacon_2","beacon_3"} );
    this->declare_parameter("initial_est",std::vector<double>{0.0,0.0,0.0} );
    this->declare_parameter("var_range",std::vector<double>{0.1,0.1,0.1,0.1,0.1} );
    this->declare_parameter("use_var",false);
    this->declare_parameter("use_var_range_proportional",false);
    
    // Read basic parameters 
    num_beacons = this->get_parameter("num_beacons").as_int();
    std::vector<double> beacons_pos_param = this->get_parameter("beacons_pos").as_double_array();
    beacons_names = this->get_parameter("beacons_names").as_string_array();
    std::vector<double> var_range_param = this->get_parameter("var_range").as_double_array();
    use_var = get_parameter("use_var").as_bool();
    use_var_range_proportional = get_parameter("use_var_range_proportional").as_bool();

    // Display Parameters
    RCLCPP_INFO_STREAM(this->get_logger(),"Num_beacons: "<< num_beacons);
    RCLCPP_INFO_STREAM(this->get_logger(),"Using beacons' variance in the meassurements: "<<std::boolalpha << use_var);
    RCLCPP_INFO_STREAM(this->get_logger(),"Using beacons' range variance proportional to distance: "<<std::boolalpha << use_var_range_proportional);
      
    // Transform std::vector params to Eigen Matrices
    Eigen::Vector2d beacon;
    var_range.resize(num_beacons);
    for (int i=0; i<num_beacons; i++)
    {
        // var of beacon_i
        var_range(i) = var_range_param[i];
        // (x,y) of beacon_i
        beacon << beacons_pos_param[2*i] , beacons_pos_param[2*i+1];
        beacons_pos.push_back(beacon);
        RCLCPP_INFO_STREAM(this->get_logger(),"Beacon "<<beacons_names[i]<<" at: [" << beacon(0)<<","<<beacon(1)<<"]");
    }
    
    // Read initial Pose of the Robot (as Eigen Matrix)
    std::vector<double> initial_est_param = this->get_parameter("initial_est").as_double_array();
    for (unsigned i=0; i<initial_est_param.size(); i++)
        est(i) = initial_est_param[i];
}

// LSE (only range) CallBack
void Lse::execute_lse_only_range(const std_msgs::msg::String::SharedPtr msg)
{
    // 0. DESERIALIZE JSON MSG
    json msg_json = json::parse(msg->data);

    // 1. INITIALIZE VARIABLES
    //-------------------------
    unsigned int iterations = 0;
    Eigen::VectorXd z(msg_json["beacons_id"].size());         // Init z: sensor of observations.
    Eigen::VectorXd dist(msg_json["beacons_id"].size());      // Init dist: estimated observations, distances to all landmakrs
    Eigen::VectorXd error(msg_json["beacons_id"].size());     // Init errors
    Eigen::Vector2d inc {{1.0,1.0}};                          // Init Step Size 
    Eigen::MatrixXd jh(msg_json["beacons_id"].size(),2);      // Init Jacobian Matrix: jh is a Num_beaconsx2 matrix
    Eigen::MatrixXd R;                                        // Init Weight Matrix R: for weighted LSE


    RCLCPP_INFO_STREAM(this->get_logger(),"Received : "<<msg_json["beacons_id"].size()<<" beacon readings, from current robot position estimation: "<<est(0)<<","<<est(1));

    // 2. LSE LOOP
    //-------------
    while (iterations < MAX_IT && inc.norm()>eps)
    {
        // 2.1 For each landmark measurement (i)
        for (long unsigned i=0; i<msg_json["beacons_id"].size(); i++)
        {
            // observation z = sensor measurement
            z(i) = msg_json["ranges"][i];

            // check the beacon that produced the observation
            auto it = std::find(beacons_names.begin(), beacons_names.end(), msg_json["beacons_id"][i]);
            int beacon_index = (it != beacons_names.end()) ? std::distance(beacons_names.begin(), it) : -1;

            // Compute the distance from the current estimated robot position (est) to beacon i
            // This can be seen as the "predicted observation"
            auto man = beacons_pos[beacon_index] - est.head(2);
            dist(i) = man.norm();
            
            // Add new row to Jacobian
            jh(i,0) = -man(0) / dist(i);
            jh(i,1) = -man(1) / dist(i);
        }

        // 2.2 Compute the error vector (measurements - predicted_observations)
        error = z-dist;
        
        // 2.3 Compute next step increment (inc)
        if (use_var)
        {   
            // Use observations' variance R 
            if (use_var_range_proportional)
                // variance is proportional to distance of landmark (as far a landmark is, as much uncertainty its reading has)
                R = (var_range.cwiseProduct(z)).asDiagonal();
            else 
                // variance is fixed (parameter)
                R = var_range.asDiagonal();

            // compute next position increment (inc)
            inc = (jh.transpose()*R.inverse()*jh).inverse()*jh.transpose()*R.inverse()*error;
        }
        else
        {
            //No variance is used:
            // compute next position increment (inc)
            inc = (jh.transpose()*jh).inverse()*jh.transpose()*error;
        }

        // 2.4 Update estimation (robot position)
        iterations++;
        est.head(2) = est.head(2) + inc;
    }


    // 3. LSE is Done!
    //-----------------    
    RCLCPP_INFO_STREAM(this->get_logger(),"LSE estimated position: "<<est(0)<<","<<est(1)<<" in "<<iterations<<" iterations");

    // Publish the robot estimated position
    if (!std::isnan(est(0)) and !std::isnan(est(1)))
    {    
        estimatedPose.position.x = est(0);
        estimatedPose.position.y = est(1);
        pub_->publish(estimatedPose);
    }
}


// Service CallBack
void Lse::handle_set_estimation(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<localization::srv::SetEstimation::Request> request,
        const std::shared_ptr<localization::srv::SetEstimation::Response> response
    )
{
    // Someone requested to update our estiamtion about the robot position
    // to a given value (x,y)
    auto x = request->estimation.position.x;
    auto y = request->estimation.position.y;
    RCLCPP_INFO_STREAM(this->get_logger(),"Received new external estimation: ("<<x<<","<<y<<")");

    // Update our internal representation
    est(0) = x;
    est(1) = y;
    
    // Publish topic
    estimatedPose.position.x = x;
    estimatedPose.position.y = y;
    pub_->publish(estimatedPose);
}


// ======== AUXILIARY FUNCTIONS ======= //
double Lse::angleDiff(double ang1, double ang2)
{
    auto dif= M_PI - fabs(fabs(angleWrap(ang1) - angleWrap(ang2)) - M_PI);
    if (ang2>ang1) dif=-dif;
    return dif;     
}


double Lse::angleWrap(double ang)
{
    while (ang > M_PI || ang<-M_PI)
    {
        if (ang>M_PI) 
            ang = ang-2*M_PI;
        else if (ang < -M_PI) 
            ang = ang+2*M_PI;
    }
    return ang;
}


// ===================================
// ============== MAIN ===============
// ===================================
int main ( int argc, char * argv[] )
{
    // init ROS2 node
    rclcpp::init ( argc, argv );
    
    // Create object from our Lse class
    auto node = std::make_shared<Lse>();
    
    // running at 5Hz
    rclcpp::Rate loop_rate(5);
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);    // attend subscriptions and srv request
        loop_rate.sleep();          // sleep till next step time
    }

    rclcpp::shutdown();
    return 0;
}