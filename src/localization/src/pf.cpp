#include <localization/pf.hpp>
#include <localization/json.hpp>

using json = nlohmann::json;

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

// Constructor
Pf::Pf(): Node ("localization_pf")
{
    // Read Node parameters
    get_parameters();

    // Mejorar el filtro de particulas para que publique la media ponderada en base a los pesos
    // Publishers & Subscribers    
    pub_ = this->create_publisher<geometry_msgs::msg::Pose> ( "/estimated_pose", 10 );
    pub_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/pf_cloud", 10);
    
    sub_ = this->create_subscription<std_msgs::msg::String>("/range_bearing_scan", 10, std::bind(&Pf::execute_pf, this, _1) );
    sub_twist_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10, std::bind(&Pf::update_pf, this, _1) );
    
    // Service Servers
    server_=this->create_service<localization::srv::SetEstimation>("set_estimation",std::bind(&Pf::handle_set_estimation,this,_1,_2,_3));

    // Init particles as matrix (x, y, phi, weight)
    p = Eigen::MatrixXd::Zero(num_particles,4);

    // Init particles 
    if (sparse == 0)
    {
        // All particles start at the initial pose estimation of the robot (est)
        p.col(0) = Eigen::VectorXd::Constant(num_particles,est(0));
        p.col(1) = Eigen::VectorXd::Constant(num_particles,est(1));
        p.col(2) = Eigen::VectorXd::Constant(num_particles,est(2));
    }
    else
    {
        // Particles are placed in a square around the initial estimation. 
        // The sparse parameter controls the size of the square
        p.col(0) = Eigen::VectorXd::Random(num_particles)*sparse + Eigen::VectorXd::Constant(num_particles,est(0));     // x
        p.col(1) = Eigen::VectorXd::Random(num_particles)*sparse + Eigen::VectorXd::Constant(num_particles,est(1));     // y
        p.col(2) = Eigen::VectorXd::Random(num_particles)*M_PI + Eigen::VectorXd::Constant(num_particles,est(2));       // phi (random value in +-pi plus the initial estimation)
        //RCLCPP_INFO_STREAM(this->get_logger(),"Particles "<<p); 
    }

    //Motion computation. We need the previous motion command     
    started = false;
    last_cmd_vel.linear.x = 0;
    last_cmd_vel.angular.z = 0;

    //Init the cloud message (for visualization in CoppeliaSIM)
    cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    // set and publish particles (variable p)
    publishCloud();
}


Pf::~Pf()
{
}


// Read parameters from yaml file
void Pf::get_parameters()
{
    // Declare parameters of this node (name, initial_value)
    this->declare_parameter("num_particles",1000);
    this->declare_parameter("num_beacons",3);
    this->declare_parameter("beacons_pos",std::vector<double>{0.0,3.0,3.0,0.0,0.0,-3.0} );
    this->declare_parameter("beacons_names",std::vector<std::string>{"beacon_1","beacon_2","beacon_3","beacon_4","beacon_5"} );
    this->declare_parameter("initial_est",std::vector<double>{0.0,0.0,0.0});
    this->declare_parameter("var_range",std::vector<double>{0.1,0.1,0.1,0.1,0.1});
    this->declare_parameter("var_bearing",std::vector<double>{0.1,0.1,0.1,0.1,0.1});
    this->declare_parameter("var_motion",std::vector<double>{0.01,0.01,0.01});
    this->declare_parameter("sparse_square",0.0);
    
    // Read parameters
    rclcpp::Parameter num_particles_param = this->get_parameter("num_particles");
    rclcpp::Parameter initial_est_param = this->get_parameter("initial_est");
    rclcpp::Parameter var_motion_param = this->get_parameter("var_motion");
    rclcpp::Parameter var_range_param = this->get_parameter("var_range");
    rclcpp::Parameter var_bearing_param = this->get_parameter("var_bearing");
    rclcpp::Parameter num_beacons_param = this->get_parameter("num_beacons");
    rclcpp::Parameter beacons_pos_param = this->get_parameter("beacons_pos");
    rclcpp::Parameter sparse_square_param = this->get_parameter("sparse_square");

    // Transform parameters to specific data types
    num_particles = (unsigned)num_particles_param.as_int();    
    beacons_names = this->get_parameter("beacons_names").as_string_array();
    num_beacons = (unsigned)num_beacons_param.as_int();
    sparse = sparse_square_param.as_double();

    // motion variances (for updating particles pose)
    for (unsigned i=0; i<var_motion_param.as_double_array().size(); i++)
        var_motion(i) = var_motion_param.as_double_array()[i];

    // Resizing vectors according to the number of beacons
    var_range.resize(num_beacons);
    var_bearing.resize(num_beacons);
    
    // Initial pose estimation    
    for (unsigned i=0; i<initial_est_param.as_double_array().size(); i++)
        est(i) = initial_est_param.as_double_array()[i];
         
    // Reading variances and beacons positions
    Eigen::Vector2d beacon;
    for (unsigned i=0; i<num_beacons; i++)
    {
        var_range(i) = var_range_param.as_double_array()[i];
        var_bearing(i) = var_range_param.as_double_array()[i];

        beacon << beacons_pos_param.as_double_array()[2*i] , beacons_pos_param.as_double_array()[2*i+1];
        beacons_pos.push_back(beacon);
        RCLCPP_INFO_STREAM(this->get_logger(),"Beacon "<<beacons_names[i]<<" at: [" << beacon(0)<<","<<beacon(1)<<"]");
    }    
    
    // Options   
    RCLCPP_INFO_STREAM(this->get_logger(),"Particle filter initialized with "<<num_particles<<" particles"); 
    RCLCPP_INFO_STREAM(this->get_logger(),"Initial estimation "<<est);
}


    


void Pf::handle_set_estimation(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<localization::srv::SetEstimation::Request> request,
        const std::shared_ptr<localization::srv::SetEstimation::Response> response
    )
    {
        //Set the particles to the given estimation, acoording to the sparse parameter
        auto x=request->estimation.position.x;
        auto y=request->estimation.position.y;
        auto z=request->estimation.orientation.z;
        RCLCPP_INFO_STREAM(this->get_logger(),"Received new external estimation: ("<<x<<","<<y<<")");
        est(0)=x;
        est(1)=y;
        est(2)=z;

        sparse=request->sparse;
        //Set the new particles' set:
        p = Eigen::MatrixXd::Zero(num_particles,4);
        if (sparse==0)
        {
            p.col(0)=Eigen::VectorXd::Constant(num_particles,est(0));
            p.col(1)=Eigen::VectorXd::Constant(num_particles,est(1));
            p.col(2)=Eigen::VectorXd::Constant(num_particles,est(2));
        }
        else
        {
            p.col(0)=Eigen::VectorXd::Random(num_particles)*sparse+Eigen::VectorXd::Constant(num_particles,est(0));
            p.col(1)=Eigen::VectorXd::Random(num_particles)*sparse+Eigen::VectorXd::Constant(num_particles,est(1));
            p.col(2)=Eigen::VectorXd::Random(num_particles)*M_PI+Eigen::VectorXd::Constant(num_particles,est(2));
            //orientation is a random value in +-pi plus the initial estimation
        }
        
        
        publishCloud();    

        estimatedPose.position.x=x;
        estimatedPose.position.y=y;
        estimatedPose.orientation.z=z;
        pub_->publish(estimatedPose);        

    }




// Update particles if the robot is moving!
// This adds uncertainty (noisy motion model)
void Pf::update_pf(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    // We move particles according to the robot movement
    // To do that, we assume the robot has kept a constant (v,w) since the last twist command
    double delta_t;
    Eigen::Vector3d pose_inc;
    rclcpp::Time now = this->get_clock()->now();

    // previous (v,w)
    auto vp = last_cmd_vel.linear.x;
    auto w = last_cmd_vel.angular.z;
    
    // the new (v,w)
    last_cmd_vel.linear.x = msg->linear.x;
    last_cmd_vel.angular.z = msg->angular.z;

    // First time calling this??
    if (!started)
    {
        started = true;
        last = now;         // update times and leave!
        return;
    }

    // Get time since last Twist (seconds)
    rclcpp::Duration diff = now-last;
    delta_t = diff.seconds();
    
    //IMPORTANT: Only update particles if the robot is moving!
    // If (v,w)=(0,0) do not update or the particles will "vibrate" due to the noise
    if (vp!=0 || w!=0) 
    {
        // Motion model (estimate inc_pose)
        if (w!=0)
              pose_inc << (vp/w)*sin( w*delta_t), (vp/w)*(1-cos(w*delta_t)), w*delta_t;
        else
              pose_inc << vp*delta_t, 0, 0;
        
        // Noise distribution
        Eigen::Vector3d res;
        Eigen::Vector3d part;
        std::default_random_engine generator;
        //normal_distribution with parameters mean and standard deviation
        std::normal_distribution<double> dist_x(0,sqrt(var_motion(0)));
        std::normal_distribution<double> dist_y(0,sqrt(var_motion(1)));
        std::normal_distribution<double> dist_theta(0,sqrt(var_motion(2)));

        // Update each particle pose with the pose_inc + noise
        for(auto row : p.rowwise())
        {
            // The noisy motion command is the pose_inc + the noise
            Eigen::Vector3d pose_inc_noisy;
            
            // Noise (different for each particle)
            pose_inc_noisy(0) = pose_inc(0) + dist_x(generator);
            pose_inc_noisy(1) = pose_inc(1) + dist_y(generator);
            pose_inc_noisy(2) = pose_inc(2) + dist_theta(generator);

            // Update each particle pose
            part << row(0),row(1),row(2);           // current particle pose (x,y,phi)
            tcomp(part,pose_inc_noisy,res);         // res = pose composition
            row << res(0),res(1),res(2),row(3);     // save result to p, maintaining its weight (row(3))
        }
    }
    // Publish the cloud over ROS2
    publishCloud();

    // Update times (for next iteration)
    last = now;
}


// Publish the poses of all particles as a ROS2 msg for visualization
void Pf::publishCloud()
{ 
    // We store the particles in variable "p" as Eigen Matrix
    // For publishing over ROS2 we create a pointCloud
    cloud_->width = 0;
    cloud_->height = 1;
    cloud_->is_dense = false;
    cloud_->points.resize(cloud_->width * cloud_->height);
    cloud_->header.frame_id = "map";
    cloud_->points.clear();
    
    // Fill pointcloud with particles data in "p"
    for(auto row : p.rowwise())
    {
            pcl::PointXYZ pt;   // new point
            pt.x = row(0);      // x
            pt.y = row(1);      // y
            pt.z = row(2);      // phi
            cloud_->points.push_back(pt);   // save point in the cloud
            cloud_->width++;
    }

    // publish PointCloud over ROS2
    sensor_msgs::msg::PointCloud2 pc2_msg_;
    pcl::toROSMsg(*cloud_, pc2_msg_);
    pc2_msg_.header.frame_id = "map";
    pc2_msg_.header.stamp = this->get_clock()->now();
    pub_cloud_->publish(pc2_msg_);
}



// New sensor measurement! 
//1. Weight particles
//2. Estimate robot pose from particles
void Pf::execute_pf(const std_msgs::msg::String::SharedPtr msg)
{
    // 0. DESERIALIZE JSON MSG (range and bearing to all landmarks)
    json msg_json = json::parse(msg->data);

    // update particles weights according to sensors' readings
    Eigen::Vector2d position_part;
    double dist, inno_range, inno_bearing;
    Eigen::Matrix2d R;

    // Use only the range&bearing reading to 1 landmark
    unsigned num_ranges = msg_json["beacons_id"].size();    // Number of beacons detected
    std::random_device rd;                                  // a seed source for the random number engine
    std::mt19937 gen(rd());                                 // mersenne_twister_engine seeded with rd()
    std::uniform_int_distribution<> distrib(0, num_ranges-1);
    unsigned selected = distrib(gen);
    RCLCPP_INFO_STREAM(this->get_logger(),"Selected beacon:" <<selected);
    
    // Get range&bearing reading from the selected beacon
    double z_range = msg_json["ranges"][selected]; 
    double z_angle = msg_json["bearings"][selected]; 

    // Covariance sensor matrix
    R << var_range(selected),0,0,var_bearing(selected);  

    // Innovation vector
    Eigen::Vector2d inno_vector;
    
    // Debug vectors to store the measured and computed distances, angle of each particle
    Eigen::VectorXd dd(num_particles);
    Eigen::VectorXd aa(num_particles);
    Eigen::VectorXd dd_m(num_particles);
    Eigen::VectorXd aa_m(num_particles);
    auto n = 0;
    //RCLCPP_INFO_STREAM(this->get_logger(),"dist:" <<z_range<<" angle "<<z_angle);

    // For each particle
    for(auto row : p.rowwise())
    {
        //1. Range Innovation
        position_part << row(0),row(1);                         // (x,y) of particle_row
        auto manhatan = beacons_pos[selected] - position_part;  // Landmark distance from particle_row
        dist = manhatan.norm();
        // range_innovation = estimated distance - observated distance
        inno_range = dist - z_range;
        
        //2. Bearing Innovation 
        //angle between each particle and the selected beacon        
        double angle = atan2(beacons_pos[selected](1)-row(1),beacons_pos[selected](0)-row(0))-row(2);
        angle = AngleWrap(angle);           //angles are in [-pi,pi]
        // bearing_innovation = estimated bearing - observated bearing
        inno_bearing = angle - z_angle;
        inno_bearing = AngleWrap(inno_bearing);

        //3. Total Innovation (range, bearing)
        inno_vector << inno_range,inno_bearing;

        //4. Compute particle weight:
        double w = inno_vector.transpose()*R.inverse()*inno_vector;
        row(3) = exp(-0.5*w) + 0.001;
        
        //RCLCPP_INFO_STREAM(this->get_logger(),"Distance:" <<dist<<","<<z_range);
        //RCLCPP_INFO_STREAM(this->get_logger(),"Angle:" <<angle<<","<<z_angle);
        //RCLCPP_INFO_STREAM(this->get_logger(),"Innov:" <<inno_range<<","<<inno_bearing);
        //RCLCPP_INFO_STREAM(this->get_logger(),"w:" <<w<<","<<row(3));

        // Store the distance and angle computed from each particle and also the received
        // meassurement for debugin
        dd(n) = dist;
        aa(n) = angle;
        dd_m(n) = z_range;
        aa_m(n) = z_angle;
        n = n+1;
    }
    
    // Resample particles according to the computed weight
    // Discrete distribution using particles' weight
    std::discrete_distribution<> discrete(p.col(3).data(), p.col(3).data()+p.col(3).size());
    
    // Reapeating sampling n times
    for (auto q=0; q<10; q++)
    {
        Eigen::MatrixXd aux = Eigen::MatrixXd::Zero(num_particles,4);
        for (unsigned n = 0; n < num_particles; ++n)
        {
            int winner = discrete(gen);  //selected index
            aux.row(n) = p.row(winner);  //update the row
        }
        p = aux;
    }
    publishCloud();


    // ESTIMATE ROBOT POSE FROM PARTICLES
    //------------------------------------
    // Getting the particle with the higher weight
    unsigned max_index;
    p.col(3).maxCoeff(&max_index);
    // Getting the average value
    auto mean_x = p.col(0).mean();
    auto mean_y = p.col(1).mean();

    double weighted_mean_x = 0;
    double weighted_mean_y = 0;
    double weights = 0;
    

    for(int i = 0; i < p.col(0).rows(); i++){
        weighted_mean_x = weighted_mean_x + p(i, 0)*p(i, 3);
        weighted_mean_y = weighted_mean_y + p(i, 1)*p(i, 3);
        weights = weights + p(i,3);
    }

    //debug: checking the goodness of the selected particle
    RCLCPP_INFO_STREAM(this->get_logger(),"--------------------");
    RCLCPP_INFO_STREAM(this->get_logger(),"Max:" <<p(max_index,0)<<","<<p(max_index,1)<<","<<p(max_index,2)<<","<<p(max_index,3));
    RCLCPP_INFO_STREAM(this->get_logger(),"Dist/Angle:" <<dd(max_index)<<","<<aa(max_index));
    RCLCPP_INFO_STREAM(this->get_logger(),"TRUTH Dist/Angle:" <<dd_m(max_index)<<","<<aa_m(max_index));
    RCLCPP_INFO_STREAM(this->get_logger(),"Innovation:" <<dd(max_index)-dd_m(max_index)<<","<<aa(max_index)-aa_m(max_index));
    
    //S et the estimated pose to the selected particle and publish it!
    /*estimatedPose.position.x = p(max_index,0);  //mean_x;
    estimatedPose.position.y = p(max_index,1);  //mean_y;//
    estimatedPose.orientation.z = p(max_index,2);*/
    if (weights <= 1e-9) {
        RCLCPP_WARN(this->get_logger(), "Weights sum is zero or too small — fallback to mean()");
        weights = 1.0;
        weighted_mean_x = p.col(0).mean();
        weighted_mean_y = p.col(1).mean();
    }


    //media del angulo
    double sum_sin = 0.0;
    double sum_cos = 0.0;
    double sum_weights = 0.0;

    for (int i = 0; i < p.rows(); ++i) {
        double w = p(i, 3);          // peso de la partícula
        double theta = p(i, 2);      // orientación de la partícula
        sum_sin = sum_sin + w * sin(theta);
        sum_cos = sum_cos + w * cos(theta);
        sum_weights = sum_weights + w;
    }

    if (sum_weights <= 1e-9) sum_weights = 1.0;  // fallback seguro

    double mean_theta = atan2(sum_sin / sum_weights, sum_cos / sum_weights);


    estimatedPose.position.x = weighted_mean_x/weights;  //mean_x;
    estimatedPose.position.y = weighted_mean_y/weights;  //mean_y;//
    estimatedPose.orientation.z = mean_theta;//p(max_index,2);
    pub_->publish(estimatedPose);
}


// ======== AUXILIARY FUNCTIONS ======= //
void Pf::tcomp(Eigen::Vector3d pose, Eigen::Vector3d inc, Eigen::Vector3d &res)
{
    res(2) = AngleWrap(pose(2)+inc(2));
    // Angles are always bounded in the interval [pi,-pi]
    auto s = sin(pose(2));
    auto c = cos(pose(2));
    res(0) = pose(0)+inc(0)*c-inc(1)*s;
    res(1) = pose(1)+inc(0)*s+inc(1)*c;
}


double Pf::AngleDiff(double ang1, double ang2)
{
    auto dif= M_PI - fabs(fabs(AngleWrap(ang1) - AngleWrap(ang2)) - M_PI);
    if (ang2>ang1) dif=-dif;
    return dif;
}


double Pf::AngleWrap(double ang)
{
    ang = fmod(ang,2.0*M_PI);

    if (ang>M_PI) ang = ang-2*M_PI;
    else if (ang < -M_PI) ang = ang+2*M_PI;
    return ang;
}


// ===================================
// ============== MAIN ===============
// ===================================
int main ( int argc, char * argv[] )
{
    // init ROS2 node
    rclcpp::init ( argc, argv );
    
    // Create object from our PF class
    auto node = std::make_shared<Pf>();

    // run at 1Hz
    rclcpp::Rate loop_rate(1);
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);    // attend subscriptions and srv request
        loop_rate.sleep();          // sleep till next step time
    }

    rclcpp::shutdown();
    return 0;
}