#ifndef DWA_PLANNER_H
#define DWA_PLANNER_H

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
class DWAPlanner : public rclcpp::Node
/**
 * @brief Class for DWA planner
 * 
 * **/
{
public:
    DWAPlanner();

    void lidar_scan_callback(const sensor_msgs::msg::LaserScan &msg);
    /**
     * Callback to handle scan message
     * **/
    void odom_callback(const nav_msgs::msg::Odometry &msg);
    /**
     * Callback to handle odometry message
     * **/
    void goal_callback(const geometry_msgs::msg::PoseStamped &msg);
    /**
     * Callback to handle goal message
     * **/
    void footprint_callback(const geometry_msgs::msg::PolygonStamped &msg);
    /**
     * Callback to handle footprint message
     * **/

    void scan_to_obstacle(const sensor_msgs::msg::LaserScan &scan);
    /**
     * @brief From scan message, calculate the position of obstacles. Update to obs_lists_
     * @param scan the scan message
     * @return The distance from robot footprint to the nearest obstacle
     * **/

protected:
    /*Member variable for neccesary information for the DWA algorithm*/
    geometry_msgs::msg::PoseStamped goal_; /*Global goal for dwa*/
    nav_msgs::msg::Odometry odom_;
    geometry_msgs::msg::PolygonStamped footprint_;
    geometry_msgs::msg::PoseArray obs_list_; /*List of observed obstacle*/



    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr footprint_sub_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
};

#endif