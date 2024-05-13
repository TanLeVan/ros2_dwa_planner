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
        Window(){};
        void show();

        double min_velocity_;
        double max_velocity_;
        double min_yawrate_;
        double max_yawrate_;

    };

    class Cost
    {
        public:
            Cost(void){};
            Cost(float obs_cost, float speed_cost, float to_goal_cost, float total_cost);
            float obs_cost{0.};
            float speed_cost{0.};
            float to_goal_cost{0.};
            float total_cost{0.};
    };

    /*Class to hold robot dynamic constrain information*/
    class RobotDynamic
    {
    public:
        double max_linear_vel_;
        double min_linear_vel_;
        double max_yaw_rate_;
        double max_acceleration_;
        double max_deceleration_;
        double max_yaw_acceleration_;
        RobotDynamic(){};
        void init(double max_linear_vel, double min_linear_vel, double max_yaw_rate, double max_acceleration,
                     double max_deceleration, double max_yaw_acceleration)
        {
            max_linear_vel_= max_linear_vel; 
            min_linear_vel_=min_linear_vel;
            max_yaw_rate_=max_yaw_rate;
            max_acceleration_=max_acceleration;
            max_deceleration_=max_deceleration;
            max_yaw_acceleration_=max_yaw_acceleration;
        }
        void show();
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

    /**Calculate and publisher velocity**/
    void cal_vel_cmd_callback(void);

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
     * return the next state of the robot assuming linear_vel and yaw_rate. The next state overwrite the state parameter.
     * **/
    void motion(State &state,const double linear_vel,const double yaw_rate);

    /**
     * Calculate the obstacle cost of a given trajectory
     * **/
    float cal_obs_cost(const std::vector<State> &traj);

    /**
     * Calculate the goal cost
     * **/
    float cal_to_goal_cost(const std::vector<State> &traj);

    /**
     * Calculate the speed cost
     *      
     **/
    float cal_speed_cost(const std::vector<State> &traj);
    /**
     * Calculate the distance to an obstacle (given as point) of a of a state of a robot
     * **/
    float cal_dist_to_obs(const geometry_msgs::msg::Pose obs, const State &state);

    /**
     * Check if obstacle point is inside of a polygom
    */
    bool is_point_inside_of_robot(const geometry_msgs::msg::Pose &obs, 
                                  const geometry_msgs::msg::PolygonStamped &footprint,
                                  const State &state);

    /**
     * Calculate the intersection point between the footprint polygon and the line connecting the 
     * origin of the  base link and the obstacle point. This function serve the purpose of finding 
     * the distance between the robot at given state and an obstacle point
     * **/
    geometry_msgs::msg::Point calculate_intersection(const geometry_msgs::msg::Pose &obs,
                                                     const geometry_msgs::msg::PolygonStamped &footprint,
                                                     const State &state);

    /**
     * Check if obstacle point is inside of a triangle 
     * **/
    bool is_point_inside_of_triangle(const geometry_msgs::msg::Pose &target_point, const geometry_msgs::msg::Polygon &triange);

    /**
     * Calculate the distance from the robot footprint 
     * to an obstacle point
     * **/
    float cal_dist_from_robot(const geometry_msgs::msg::Pose &obs, const geometry_msgs::msg::PolygonStamped &footprint, const State &state);

    /**
     * Move the footprint to the given state by finding the corresponding coordinate
     * of each point in the footprint if the robot were to be at the given state.
     * The coorinate frame of the footprint is the coordinate frame of the state.
     * **/
    inline void move_footprint(const State &state, geometry_msgs::msg::PolygonStamped &footprint);
    
    /**
     * Check if the robot reach the goal
     * **/
    bool is_goal_reached(void);
protected:
    /*Member variable for neccesary information for the DWA algorithm*/
    std::string robot_frame_{"base_link"};
    geometry_msgs::msg::PoseStamped goal_; /*Global goal for dwa. With respect to robot_frame_*/
    nav_msgs::msg::Odometry odom_;
    geometry_msgs::msg::PolygonStamped footprint_;
    geometry_msgs::msg::PoseArray obs_list_; /*List of observed obstacle*/
    geometry_msgs::msg::Twist current_vel_; 
    RobotDynamic robot_dynamic_;

    bool is_scan_updated_{false};
    bool is_goal_updated_{false};
    bool is_footprint_updated_{false};
    bool is_odom_updated_{false};

    /*Parameter to change the behaviour of the DWA algorithm*/
    int linear_vel_sample_size_{4}; 
    int yaw_rate_sample_size_{4};
    double sim_time_{3}; //in second 
    double sim_timestep_{0.5}; // in second
    double scan_range_{1}; //in meter
    int skip_point{5}; //How many point to skip when converting lidar scan to obstacle
    bool use_footprint_{false};
    double circle_footprint_radius{0.15}; //in meter 
    double distance_to_goal_threshold_{0.1}; //in meter

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr footprint_sub_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};

    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr obs_list_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;

    rclcpp::TimerBase::SharedPtr timer_;
 };

#endif