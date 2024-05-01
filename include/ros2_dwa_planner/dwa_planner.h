#ifndef DWA_PLANNER_H
#define DWA_PLANNER_H

#include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>
#include <utility>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/twist.hpp>

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

    class State
    /**
     * Class to store the state of the robot.
     * **/
    {
        public: 
        State();
        State(const double x, const double y, const double yaw, const double velocity, const double yawrate);

        double x_; 
        double y_; 
        double yaw_;
        double velocity_;
        double yawrate_;
    };

    class Window
    {
        public:
        Window();
        void show();

        double min_velocity_;
        double max_velocity_;
        double min_yawrate_;
        double max_yawrate_;

    };

    class Cost
    {
        public:
            float obs_cost;
            float speed_cost;
            float to_goal_cost;
    };

    /*Class to hold robot dynamic constrain information*/
    class RobotDynamic
    {
    public:
        float max_linear_vel_;
        float min_linear_vel_;
        float max_yaw_rate_;
        float max_acceleration_;
        float max_deceleration_;
        float max_yaw_acceleration_;
        RobotDynamic(){};
        void init(float max_linear_vel, float min_linear_vel, float max_yaw_rate, float max_acceleration,
                     float max_deceleration, float max_yaw_acceleration)
        {
            max_linear_vel_= max_linear_vel; 
            min_linear_vel_=min_linear_vel;
            max_yaw_rate_=max_yaw_rate;
            max_acceleration_=max_acceleration;
            max_deceleration_=max_deceleration;
            max_yaw_acceleration_=max_yaw_acceleration;
        }
    };

    /**
     * Callback to handle scan message
     * **/
    void lidar_scan_callback(const sensor_msgs::msg::LaserScan &msg);

    /**
     * Callback to handle odometry message
     * **/
    void odom_callback(const nav_msgs::msg::Odometry &msg);

    /**
     * Callback to handle goal message
     * **/
    void goal_callback(const geometry_msgs::msg::PoseStamped &msg);

    /**
     * Callback to handle footprint message
     * **/
    void footprint_callback(const geometry_msgs::msg::PolygonStamped &msg);

    /**
     * @brief From scan message, calculate the position of obstacles. Update to obs_lists_
     * @param scan the scan message
     * @return The distance from robot footprint to the nearest obstacle
     * **/
    void scan_to_obstacle(const sensor_msgs::msg::LaserScan &scan);

    /**
     * Check if any obstacle collide with a given state of the robot. 
     * **/
    void check_collision(const State &state);

    /**
     * Beginning the DWA algorithm. Returning the velocity command that the robot should follow
     * **/
    geometry_msgs::msg::Twist DWA_process(void);

    /**
     * Calculate the dynamic window from the current
     * **/
    Window cal_dynamic_window(void);

    /**
     * Create limited pair of linear angular velocity from the dynammic window 
    **/
    std::vector<std::pair<double, double>> discretize_dynamic_window(const Window &window);

    /**
     * Generate trajectory from linear and angular velocity for sim_time_ in the future
     * **/
    std::vector<State> generate_trajectory(const double linear_vel, const double yaw_rate);

    /**
     * Evaluate the cost of a given trajectory
     * **/
    Cost evaluate_trajectory(const std::vector<State> &traj);

    /**
     * Calculate the next state of the robot after one time step
     * @param state reference to the current state of the robot
     * @param linear_vel the assumed linear velocity
     * @param yaw_rate the assumed yaw rate
     * @return the next state of the robot assuming linear_vel and yaw_rate. The next state overwrite the state parameter.
     * **/
    void motion(State &state,const double linear_vel,const double yaw_rate);

    /**
     * Calculate the obstacle cost of a given trajectory
     * **/
    float cal_obs_cost(std::vector<State> &traj);

    /**
     * Calculate the distance to an obstacle (given as point) of a trajectory
     * **/
    float cal_dist_to_obs(const std::vector<State> &traj, const geometry_msgs::msg::Pose obs);

    /**
     * Move the footprint of the robot from the current state to a given state
     * **/
    geometry_msgs::msg::PolygonStamped move_footprint(const State &state);

    /**
     * Check if obstacle point is inside of robot
    */
    bool is_point_inside_of_robot(const geometry_msgs::msg::Pose &obs, 
                                  const geometry_msgs::msg::PolygonStamped &footprint,
                                  const State &state);


protected:
    /*Member variable for neccesary information for the DWA algorithm*/
    std::string robot_frame_{"/base_link"};
    geometry_msgs::msg::PoseStamped goal_; /*Global goal for dwa. With respect to robot_frame_*/
    nav_msgs::msg::Odometry odom_;
    geometry_msgs::msg::PolygonStamped footprint_;
    geometry_msgs::msg::PoseArray obs_list_; /*List of observed obstacle*/
    geometry_msgs::msg::Twist current_vel_; 
    RobotDynamic robot_dynamic_;

    /*Parameter to change the behaviour of the DWA algorithm*/
    int linear_vel_sample_size_;
    int yaw_rate_sample_size_;
    double sim_time_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr footprint_sub_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
};

#endif