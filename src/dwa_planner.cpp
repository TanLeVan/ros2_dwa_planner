#include "ros2_dwa_planner/dwa_planner.h"
#include <iostream>


#define USE_TEST true

DWAPlanner::DWAPlanner()
    : Node("DWAPlanner_node")
{
    std::cout << "Instantiate DWA planner node" << std::endl;

    
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
}

void DWAPlanner::lidar_scan_callback(const sensor_msgs::msg::LaserScan &msg)
{
    scan_to_obstacle(msg);
}

void DWAPlanner::footprint_callback(const geometry_msgs::msg::PolygonStamped &msg)
{

}

void DWAPlanner::odom_callback(const nav_msgs::msg::Odometry &msg)
{

}

void DWAPlanner::scan_to_obstacle(const sensor_msgs::msg::LaserScan &scan)
{
    /**
     * Getting the position of obstacle in (x, y) coordinate with respect to the lidar
     * **/
    obs_list_.poses.clear();
    float angle{scan.angle_min};
    for (auto r : scan.ranges)
    {
        geometry_msgs::msg::Pose pose;
        pose.position.x = r * cos(angle);
        pose.position.y = r * sin(angle);
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