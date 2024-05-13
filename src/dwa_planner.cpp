#include "ros2_dwa_planner/dwa_planner.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "builtin_interfaces/msg/time.hpp"
#include "tf2/time.h"

#include <Eigen/Dense>
#include <iostream>
#include <cfloat>
#include <chrono> 

#define USE_TEST true
#define DEGREE_TO_RAD 3.14159265359/180
using namespace std::chrono_literals;

DWAPlanner::DWAPlanner()
    : Node("DWAPlanner_node")
{
    std::cout << "Instantiate DWA planner node" << std::endl;
    /*Initializing parameters*/
    this->declare_parameter("MAX_LINEAR_VEL", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("MIN_LINEAR_VEL",rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("MAX_YAW_RATE",rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("MAX_ACCELERATION",rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("MAX_DECELERATION", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("MAX_YAW_ACCELERATION", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("LINEAR_VEL_SAMPLE_SIZE", rclcpp::PARAMETER_INTEGER);
    this->declare_parameter("YAW_RATE_SAMPLE_SIZE",rclcpp::PARAMETER_INTEGER);
    this->declare_parameter("SIM_TIME",rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("SIM_TIMESTEP",rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("SCAN_RANGE", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("SKIP_POINT", rclcpp::PARAMETER_INTEGER);
    this->declare_parameter("DISTANCE_TO_GOAL_THRESHOLD", rclcpp::PARAMETER_DOUBLE);

    linear_vel_sample_size_ = this->get_parameter("LINEAR_VEL_SAMPLE_SIZE").as_int(); 
    yaw_rate_sample_size_ = this->get_parameter("YAW_RATE_SAMPLE_SIZE").as_int();
    sim_time_ = this->get_parameter("SIM_TIME").as_double(); //in second 
    sim_timestep_ = this->get_parameter("SIM_TIMESTEP").as_double(); // in second
    scan_range_ = this->get_parameter("SCAN_RANGE").as_double(); //in meter
    skip_point = this->get_parameter("SKIP_POINT").as_int(); //How many point to skip when converting lidar scan to obstacle
    distance_to_goal_threshold_ = this->get_parameter("DISTANCE_TO_GOAL_THRESHOLD").as_double();
    if(!use_footprint_)
        is_footprint_updated_ = true;

    /*Initializing robot dynamic constraint*/
    robot_dynamic_.init(this->get_parameter("MAX_LINEAR_VEL").as_double(),
                        this->get_parameter("MIN_LINEAR_VEL").as_double(),
                        this->get_parameter("MAX_YAW_RATE").as_double()*DEGREE_TO_RAD,
                        this->get_parameter("MAX_ACCELERATION").as_double(),
                        this->get_parameter("MAX_DECELERATION").as_double(),
                        this->get_parameter("MAX_YAW_ACCELERATION").as_double()*DEGREE_TO_RAD);
    
    robot_dynamic_.show();
    std::cout << "Max yaw rate: " <<this->get_parameter("MAX_YAW_RATE").as_double() << std::endl;

    /*Initializing tf listener*/
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    /*Initializing subscriber*/
    lidar_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", 10, std::bind(&DWAPlanner::lidar_scan_callback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 10, std::bind(&DWAPlanner::odom_callback, this, std::placeholders::_1));
    footprint_sub_ = this->create_subscription<geometry_msgs::msg::PolygonStamped>(
    "/footprint", 10, std::bind(&DWAPlanner::footprint_callback, this, std::placeholders::_1));
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/goal", 10, std::bind(&DWAPlanner::goal_callback, this, std::placeholders::_1));

    /*Initializing publisher*/
    obs_list_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/obstacle_list", 10);
    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    /*Publishing velocity command*/
    timer_ = this->create_wall_timer(500ms,std::bind(&DWAPlanner::cal_vel_cmd_callback, this));
}

DWAPlanner::State::State(const double x, const double y, const double yaw, const double velocity, const double yawrate)
    : x_{x}, y_{y}, yaw_{yaw}, velocity_{velocity}, yawrate_{yawrate}   
{
}

DWAPlanner::Cost::Cost(float obs_cost, float speed_cost, float to_goal_cost, float total_cost)
    : obs_cost{obs_cost}, speed_cost{speed_cost}, to_goal_cost{to_goal_cost}, total_cost{total_cost}
{
}

void DWAPlanner::Window::show(){
    std::cout  << max_velocity_ << std::endl;
    std::cout  << min_velocity_ << std::endl;
    std::cout  << max_yawrate_ << std::endl;
    std::cout  << min_yawrate_ << std::endl;
}
void DWAPlanner::RobotDynamic::show()
{
    std::cout << "Max linear vel: "<<max_linear_vel_ <<std::endl;
    std::cout << "Min linear vel: "<<min_linear_vel_ <<std::endl;
    std::cout << "Max yaw rate: "<<max_yaw_rate_ <<std::endl;
    std::cout << "Max acceleration: "<<max_acceleration_ <<std::endl;
    std::cout << "Max deceleration: " <<max_deceleration_ <<std::endl;
    std::cout << "Max yaw acceleration: " << max_yaw_acceleration_ << std::endl;
}

void DWAPlanner::cal_vel_cmd_callback(void)
{
    auto start = std::chrono::high_resolution_clock::now();
    if(is_goal_reached())
    {
        geometry_msgs::msg::Twist vel_command;
        vel_command.linear.y = 0;
        vel_command.linear.z = 0;
        vel_command.angular.x = 0;
        vel_command.angular.y = 0;        
        vel_command.linear.x = 0;
        vel_command.angular.z = 0;
        vel_pub_->publish(vel_command);
    }
    else if(is_goal_updated_ && is_odom_updated_ && is_scan_updated_ && is_footprint_updated_)
    {
        std::vector<std::pair<std::vector<State>, Cost>>  traj_with_cost_list;
        Cost min_cost{0., 0., 0., 1e6};
        std::vector<State> best_traj;
        best_traj.reserve(static_cast<int>(sim_time_/sim_timestep_));
        geometry_msgs::msg::Twist vel_command;
        vel_command.linear.y = 0;
        vel_command.linear.z = 0;
        vel_command.angular.x = 0;
        vel_command.angular.y = 0;

        std::cout << "Begin calculating best velocity " << "\n";
        Window dynamic_window = cal_dynamic_window();
        std::vector<std::pair<double, double>> vel_pair_list = discretize_dynamic_window(dynamic_window);
        for (auto vel_pair : vel_pair_list)
        {
            std::vector<State> traj = generate_trajectory(vel_pair.first, vel_pair.second);
            Cost cost = evaluate_trajectory(traj);
            traj_with_cost_list.push_back(std::make_pair(traj, cost));
        }
        for (auto traj_with_cost : traj_with_cost_list)
        {
            if (traj_with_cost.second.total_cost < min_cost.total_cost)
            {
                min_cost.total_cost = traj_with_cost.second.total_cost;
                best_traj = traj_with_cost.first;
            }
        }
        vel_command.linear.x = best_traj.front().velocity_;
        vel_command.angular.z = best_traj.front().yawrate_;
        vel_pub_->publish(vel_command);

        std::cout << "Finish calculating best velocity " << "\n";
        is_goal_updated_ = false;
        is_odom_updated_ = false;
        is_scan_updated_ = false;
        if(use_footprint_)
            is_footprint_updated_ = false;
    }
    auto stop= std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    std::cout << "Find velocity execution time    " << duration.count()*1e-6 << "s"<< "\n";
    std::cout << "\n\n";
}

void DWAPlanner::lidar_scan_callback(const sensor_msgs::msg::LaserScan &msg)
{
    if(!is_scan_updated_)
    {
        scan_to_obstacle(msg);
    }
}

void DWAPlanner::footprint_callback(const geometry_msgs::msg::PolygonStamped &msg)
/**Receive footprint topic and transform to robot frame**/
{
    if(use_footprint_)
    {
        footprint_ = msg;
        if (footprint_.header.frame_id != robot_frame_)
        {
            try {
                geometry_msgs::msg::TransformStamped to_robot_frame = tf_buffer_->lookupTransform(
                    robot_frame_, footprint_.header.frame_id,
                    tf2::TimePointZero);
                tf2::doTransform(footprint_, footprint_, to_robot_frame);

            } catch (const tf2::TransformException & ex) {
                RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s",
                robot_frame_.c_str(), footprint_.header.frame_id.c_str(), ex.what());    
            return;
            }
        }
        is_footprint_updated_ = true;
    }
}

void DWAPlanner::odom_callback(const nav_msgs::msg::Odometry &msg)
{
    if (!is_odom_updated_)
    {
        current_vel_ = msg.twist.twist;
        is_odom_updated_ = true;
    }
}

void DWAPlanner::goal_callback(const geometry_msgs::msg::PoseStamped &msg)
/**Receive goal message. Transform goal message to robot_frame_**/
{
    if(!is_goal_updated_)
    {
        goal_ = msg;
        if (goal_.header.frame_id != robot_frame_)
        {
            try {
                geometry_msgs::msg::TransformStamped to_robot_frame = tf_buffer_->lookupTransform(
                    robot_frame_, goal_.header.frame_id,
                    tf2::TimePointZero);
                tf2::doTransform(goal_, goal_, to_robot_frame);
                std::cout << "Goal x after transforming " << goal_.pose.position.x << std::endl;

            } catch (const tf2::TransformException & ex) {
                RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s",
                robot_frame_.c_str(), goal_.header.frame_id.c_str(), ex.what());    
            return;
            }
        }
        is_goal_updated_ = true;
    }
}

void DWAPlanner::scan_to_obstacle(const sensor_msgs::msg::LaserScan &scan)
{
    /**
     * Getting the position of obstacle in (x, y) coordinate with respect to the lidar
     * coordinate
     * **/
    obs_list_.header.frame_id = robot_frame_;
    obs_list_.header.stamp = scan.header.stamp;
    obs_list_.poses.clear();
    std::string lidar_frame{scan.header.frame_id};
    float angle{scan.angle_min};
    for (std::size_t i = 0; i<scan.ranges.size(); i+=skip_point)
    {
        float r = scan.ranges[i];
        /*Ignore r out of scan range*/
        if (!std::isfinite(r) || r > scan_range_)
        { 
            angle += scan.angle_increment*skip_point; 
            continue;
        }
        geometry_msgs::msg::Pose pose;
        pose.position.x = r * cos(angle);
        pose.position.y = r * sin(angle);
        angle += scan.angle_increment*skip_point; 
        /**Tranform the pose from lidar coordinate to robot coordinate (/base_link)**/
        try {
            geometry_msgs::msg::TransformStamped to_robot_frame = tf_buffer_->lookupTransform(
                robot_frame_, lidar_frame,
                tf2::TimePointZero);
            tf2::doTransform(pose, pose, to_robot_frame);
        } catch (const tf2::TransformException & ex) {
            RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s",
            robot_frame_.c_str(), lidar_frame.c_str(), ex.what());    
        return;
        }
        obs_list_.poses.push_back(pose);
    }
    is_scan_updated_ = true;
    if(USE_TEST)
    {
        std::cout << "Converting scan to obstacle" << std::endl;
        obs_list_pub_->publish(obs_list_);
    }
}

DWAPlanner::Window DWAPlanner::cal_dynamic_window(void)
{
    DWAPlanner::Window window;
    window.min_velocity_ = std::max(current_vel_.linear.x - robot_dynamic_.max_deceleration_*sim_timestep_ , robot_dynamic_.min_linear_vel_);
    window.max_velocity_= std::min(current_vel_.linear.x + robot_dynamic_.max_acceleration_*sim_timestep_ , robot_dynamic_.max_linear_vel_);
    window.min_yawrate_ = std::max(current_vel_.angular.z - robot_dynamic_.max_yaw_acceleration_*sim_timestep_, -robot_dynamic_.max_yaw_rate_);
    window.max_yawrate_ = std::min(current_vel_.angular.z + robot_dynamic_.max_yaw_acceleration_*sim_timestep_, robot_dynamic_.max_yaw_rate_);
    return window;
}

std::vector<std::pair<double, double>> DWAPlanner::discretize_dynamic_window(const DWAPlanner::Window &window)
{
    double dv = (window.max_velocity_-window.min_velocity_)/(linear_vel_sample_size_-1); //Linear velocity step
    double dw = (window.max_yawrate_-window.min_yawrate_)/(yaw_rate_sample_size_-1); //Angular velocity step
    std::vector<std::pair<double, double>> vel_pair_list;  //pair of linear and angular velocity
    vel_pair_list.reserve(linear_vel_sample_size_*(yaw_rate_sample_size_+1)); 
    for (int i = 0;i<linear_vel_sample_size_;++i)
    {
        double linear_vel{window.min_velocity_ + i*dv};
        for (int j = 0; j<yaw_rate_sample_size_;++j)
        {
            double yaw_rate{window.min_yawrate_ + j*dw};
            vel_pair_list.push_back(std::make_pair(linear_vel, yaw_rate));
        }

        //Consider straight trajectory
        vel_pair_list.push_back(std::make_pair(linear_vel,0.));
    }
    return vel_pair_list;
}

std::vector<DWAPlanner::State> DWAPlanner::generate_trajectory(const double linear_vel, const double yaw_rate)
{
    int sim_time_sample_size = static_cast<int>(sim_time_/sim_timestep_);
    std::vector<State> trajectory;
    trajectory.reserve(sim_time_sample_size);
    State state(0., 0., 0., linear_vel, yaw_rate);
    for (int i = 0; i<sim_time_sample_size;++i)
    {
        motion(state, linear_vel, yaw_rate);
        trajectory.push_back(state);      
    }
    return trajectory;
}

void DWAPlanner::motion(DWAPlanner::State &state, double linear_vel, double yaw_rate)
{
    state.yaw_ += sim_timestep_*yaw_rate;
    state.x_ += linear_vel*std::cos(state.yaw_)*sim_timestep_;
    state.y_ += linear_vel*std::sin(state.yaw_)*sim_timestep_;
    state.velocity_ =  linear_vel;
    state.yawrate_ = yaw_rate;
}

DWAPlanner::Cost DWAPlanner::evaluate_trajectory(const std::vector<DWAPlanner::State> &traj)
{
    Cost cost;
    cost.to_goal_cost = cal_to_goal_cost(traj);
    cost.speed_cost = cal_speed_cost(traj);
    cost.obs_cost = cal_obs_cost(traj);
    cost.total_cost  = 3*cost.to_goal_cost + 3*cost.obs_cost + 5*cost.speed_cost;
    return cost;
}

float DWAPlanner::cal_speed_cost(const std::vector<State> &traj)
{  
    const Window window = cal_dynamic_window();
    return window.max_velocity_ - traj.front().velocity_;
}

float DWAPlanner::cal_to_goal_cost(const std::vector<State> &traj)
{
    tf2::Quaternion q{goal_.pose.orientation.x,
                      goal_.pose.orientation.y,
                      goal_.pose.orientation.z,
                      goal_.pose.orientation.w};
    tf2::Matrix3x3 m{q};
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    const Eigen::Vector3d goal(goal_.pose.position.x, goal_.pose.position.y, yaw);
    Eigen::Vector3d last_position(traj.back().x_,traj.back().y_, traj.back().yaw_);
    std::cout << "Goal cost is " << (last_position.segment(0,2) - goal.segment(0,2)).norm() << "\n";
    return (last_position.segment(0,2) - goal.segment(0,2)).norm();
}

float DWAPlanner::cal_obs_cost(const std::vector<State> &traj)
{
    // /auto start = std::chrono::high_resolution_clock::now();
    geometry_msgs::msg::PolygonStamped footprint;
    float min_dist{1e6};
    float dist; // Distance from one state to one obstacle
    for (auto state : traj)
    { 
        if(use_footprint_)
        {
            move_footprint(state, footprint);
            for(const geometry_msgs::msg::Pose &obs : obs_list_.poses)
            {
                dist = cal_dist_from_robot(obs, footprint, state);
                min_dist = std::min(dist, min_dist);
            }
        }
        else
        {
            for(const geometry_msgs::msg::Pose &obs : obs_list_.poses)
            {
                dist = hypot((obs.position.x - state.x_), (obs.position.y - state.y_)) - circle_footprint_radius; 
                min_dist = std::min(dist, min_dist);
            }
        }
    }
    // auto stop= std::chrono::high_resolution_clock::now();
    // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    // std::cout << "cal_obs_cost execution time for 1 traj     " << duration.count()*1e-6 <<"s" << "\n";

    if (min_dist < DBL_EPSILON)
    {
        std::cout << "Trajectory COLLIDE" << "\n";
        return 1e6;
    }
    std::cout << "Obs cost is " << scan_range_ - min_dist <<  "\n";
    return scan_range_-min_dist;
}

float DWAPlanner::cal_dist_from_robot(const geometry_msgs::msg::Pose &obs, const geometry_msgs::msg::PolygonStamped &footprint, const DWAPlanner::State &state)
{
    if (is_point_inside_of_robot(obs, footprint, state))
    {
        return 0.0;
    }
    else
    {
        geometry_msgs::msg::Point intersection = calculate_intersection(obs, footprint, state);
        return hypot((intersection.x - obs.position.x), (intersection.y - obs.position.y));
    }
}

inline void DWAPlanner::move_footprint(const State &state, geometry_msgs::msg::PolygonStamped &footprint)
{
    // footprint.header.stamp = this->get_clock()->now();
    footprint = footprint_;
    for (auto &point : footprint.polygon.points)
    {
        Eigen::VectorXf point_in(2);
        point_in << point.x, point.y;
        Eigen::Matrix2f rot;
        rot = Eigen::Rotation2Df(state.yaw_);
        const Eigen::VectorXf point_out = rot * point_in;

        point.x = point_out.x() + state.x_;
        point.y = point_out.y() + state.y_;
    }
}


bool DWAPlanner::is_point_inside_of_robot(const geometry_msgs::msg::Pose &obs, 
                                          const geometry_msgs::msg::PolygonStamped &footprint,
                                          const State &state)
{
    geometry_msgs::msg::Point32 state_point;
    state_point.x = state.x_;
    state_point.y = state.y_;
    for (std::size_t i = 0; i < footprint.polygon.points.size();++i)
    {
        /*create triangle from center point of robot (state_point) and 2 consecutive point 
        in the footprint. Do this for all point in footprint*/
        geometry_msgs::msg::Polygon triangle;
        triangle.points.push_back(state_point);
        triangle.points.push_back(footprint.polygon.points[i]);
        //If point is not last point
        if(i != footprint.polygon.points.size() - 1)
            //Create triangle with next point
            triangle.points.push_back(footprint.polygon.points[i+1]);
        //if point is last point
        else
            //create triangle with first point
            triangle.points.push_back(footprint.polygon.points[0]);

        /*check if obstacle point is inside the triangle*/
        if(is_point_inside_of_triangle(obs, triangle))
            return true;
    }
    return false;
}

bool DWAPlanner::is_point_inside_of_triangle(const geometry_msgs::msg::Pose &target_point, const geometry_msgs::msg::Polygon &triangle)
{
    if (triangle.points.size() != 3)
    {
        exit(1);
    }
    const Eigen::Vector3d vector_A(triangle.points[0].x, triangle.points[0].y, 0.0);
    const Eigen::Vector3d vector_B(triangle.points[1].x, triangle.points[1].y, 0.0);
    const Eigen::Vector3d vector_C(triangle.points[2].x, triangle.points[2].y, 0.0);
    const Eigen::Vector3d vector_P(target_point.position.x, target_point.position.y, 0.0);

    const Eigen::Vector3d vector_AB = vector_B - vector_A;
    const Eigen::Vector3d vector_BP = vector_P - vector_B;
    const Eigen::Vector3d cross1 = vector_AB.cross(vector_BP);

    const Eigen::Vector3d vector_BC = vector_C - vector_B;
    const Eigen::Vector3d vector_CP = vector_P - vector_C;
    const Eigen::Vector3d cross2 = vector_BC.cross(vector_CP);

    const Eigen::Vector3d vector_CA = vector_A - vector_C;
    const Eigen::Vector3d vector_AP = vector_P - vector_A;
    const Eigen::Vector3d cross3 = vector_CA.cross(vector_AP);

  if ((0 < cross1.z() && 0 < cross2.z() && 0 < cross3.z()) || (cross1.z() < 0 && cross2.z() < 0 && cross3.z() < 0))
    return true;
  else
    return false;
}

geometry_msgs::msg::Point DWAPlanner::calculate_intersection(const geometry_msgs::msg::Pose &obs,
                                                     const geometry_msgs::msg::PolygonStamped &footprint,
                                                     const State &state)
{
    for (std::size_t i = 0; i < footprint.polygon.points.size(); ++i)
    {
        const Eigen::Vector3d vector_A(obs.position.x, obs.position.y, 0.0);
        const Eigen::Vector3d vector_B(state.x_, state.y_, 0.0);
        const Eigen::Vector3d vector_C(footprint.polygon.points[i].x, footprint.polygon.points[i].y, 0.0);
        Eigen::Vector3d vector_D(0.0, 0.0, 0.0);
        if (i != footprint.polygon.points.size() - 1)
            vector_D << footprint.polygon.points[i + 1].x, footprint.polygon.points[i + 1].y, 0.0;
        else
            vector_D << footprint.polygon.points[0].x, footprint.polygon.points[0].y, 0.0;

        const double deno = (vector_B - vector_A).cross(vector_D - vector_C).z();
        const double s = (vector_C - vector_A).cross(vector_D - vector_C).z() / deno;
        const double t = (vector_B - vector_A).cross(vector_A - vector_C).z() / deno;

        geometry_msgs::msg::Point point;
        point.x = vector_A.x() + s * (vector_B - vector_A).x();
        point.y = vector_A.y() + s * (vector_B - vector_A).y();

        // cross
        if (!(s < 0.0 || 1.0 < s || t < 0.0 || 1.0 < t))
        {  
            return point;
        }
    }

    geometry_msgs::msg::Point point;
    point.x = 1e6;
    point.y = 1e6;
    return point;
}

bool DWAPlanner::is_goal_reached()
{
    if(is_goal_updated_)
    {
        if(hypot(goal_.pose.position.x, goal_.pose.position.y) < distance_to_goal_threshold_)
        {
            RCLCPP_INFO(this->get_logger(), "Goal reached");
            return true;
        }
    }
    return false;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DWAPlanner>());
    rclcpp::shutdown();
    return 0;
}