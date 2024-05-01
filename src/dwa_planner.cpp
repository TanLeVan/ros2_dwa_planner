#include "ros2_dwa_planner/dwa_planner.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <iostream>


#define USE_TEST true

DWAPlanner::DWAPlanner()
    : Node("DWAPlanner_node")
{
    std::cout << "Instantiate DWA planner node" << std::endl;

    this->declare_parameter("MAX_LINEAR_VEL", 1.);
    this->declare_parameter("MIN_LINEAR_VEL", 0.);
    this->declare_parameter("MAX_YAW_RATE", 90);
    this->declare_parameter("MAX_ACCELERATION", 1.);
    this->declare_parameter("MAX_DECELERATION", 1.);
    this->declare_parameter("MAX_YAW_ACCELERATION", 45);

    /*Initializing robot dynamic constraint*/
    robot_dynamic_.init(this->get_parameter("MAX_LINEAR_VEL").as_double(),
                        this->get_parameter("MIN_LINEAR_VEL").as_double(),
                        this->get_parameter("MAX_YAW_RATE").as_double(),
                        this->get_parameter("MAX_ACCELERATION").as_double(),
                        this->get_parameter("MAX_DECELERATION").as_double(),
                        this->get_parameter("MAX_YAW_ACCELERATION").as_double());
    

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
}

void DWAPlanner::lidar_scan_callback(const sensor_msgs::msg::LaserScan &msg)
{
    scan_to_obstacle(msg);
}

void DWAPlanner::footprint_callback(const geometry_msgs::msg::PolygonStamped &msg)
/**Receive footprint topic and transform to robot frame**/
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

}

void DWAPlanner::odom_callback(const nav_msgs::msg::Odometry &msg)
{
    current_vel_ = msg.twist.twist;
}

void DWAPlanner::goal_callback(const geometry_msgs::msg::PoseStamped &msg)
/**Receive goal message. Transform goal message to robot_frame_**/
{
    goal_ = msg;
    if (goal_.header.frame_id != robot_frame_)
    {
        try {
            geometry_msgs::msg::TransformStamped to_robot_frame = tf_buffer_->lookupTransform(
                robot_frame_, goal_.header.frame_id,
                tf2::TimePointZero);
            tf2::doTransform(goal_, goal_, to_robot_frame);

        } catch (const tf2::TransformException & ex) {
            RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s",
            robot_frame_.c_str(), goal_.header.frame_id.c_str(), ex.what());    
          return;
        }
    }
}

void DWAPlanner::scan_to_obstacle(const sensor_msgs::msg::LaserScan &scan)
{
    /**
     * Getting the position of obstacle in (x, y) coordinate with respect to the lidar
     * coordinate
     * **/
    obs_list_.poses.clear();
    std::string lidar_frame{scan.header.frame_id};
    float angle{scan.angle_min};
    for (auto r : scan.ranges)
    {
        geometry_msgs::msg::Pose pose;
        pose.position.x = r * cos(angle);
        pose.position.y = r * sin(angle);
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
        angle += scan.angle_increment;
    }
    if(USE_TEST)
    {
        std::cout << "Converted scan to obstacle" << std::endl;
    }
}



int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DWAPlanner>());
    rclcpp::shutdown();
    return 0;
}