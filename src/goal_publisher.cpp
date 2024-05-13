#include <chrono>
#include <functional>
#include <string>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include "tf2/LinearMath/Quaternion.h"
#include <rosgraph_msgs/msg/clock.hpp>

#define DEGREE_TO_RAD 3.14159265359/180

using namespace std::chrono_literals;
using std::placeholders::_1;

class GoalPublisher : public rclcpp::Node
{
public:
    GoalPublisher()
    : Node("goal_publisher")
    {
        //The line below is responsible for creating parameter and set default value
        this->declare_parameter("goal_yaw_in_degree",rclcpp::PARAMETER_DOUBLE);
        this->declare_parameter("goal_x", rclcpp::PARAMETER_DOUBLE);
        this->declare_parameter("goal_y", rclcpp::PARAMETER_DOUBLE);
        this->declare_parameter("goal_frame", rclcpp::PARAMETER_STRING);
        this->declare_parameter("sim_time", false);
        use_sim_time_ = this->get_parameter("sim_time").as_bool();

        goal_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal", 10);
        if(use_sim_time_)
        {
            clock_sub_ = this->create_subscription<rosgraph_msgs::msg::Clock>("/clock", rclcpp::SensorDataQoS(), 
                            std::bind(&GoalPublisher::clock_callback, this, _1));
        }
        timer_ = this->create_wall_timer(
            500ms, std::bind(&GoalPublisher::timer_callback, this));
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_publisher_;
    rclcpp::Subscription<rosgraph_msgs::msg::Clock> ::SharedPtr clock_sub_;
    geometry_msgs::msg::PoseStamped goal_;
    rosgraph_msgs::msg::Clock sim_clock_;
    bool use_sim_time_;
    bool clock_update_{false};

    void timer_callback()
    {
        if(use_sim_time_)
        {
            goal_.header.stamp.sec = sim_clock_.clock.sec;
            goal_.header.stamp.nanosec = sim_clock_.clock.nanosec;
        }
        else
        {
            goal_.header.stamp = this->get_clock()->now();
        }
        goal_.header.frame_id = this->get_parameter("goal_frame").as_string();

        goal_.pose.position.x = this->get_parameter("goal_x").as_double();
        goal_.pose.position.y = this->get_parameter("goal_y").as_double();
        goal_.pose.position.z = 0.;
        tf2::Quaternion q;
        q.setRPY(0,0,this->get_parameter("goal_yaw_in_degree").as_double()*DEGREE_TO_RAD);
        goal_.pose.orientation.x = q.x();
        goal_.pose.orientation.y = q.y();
        goal_.pose.orientation.z = q.z();
        goal_.pose.orientation.w = q.w();

        // RCLCPP_INFO(this->get_logger(), "Now is: %d!", goal_.header.stamp.sec);
        // RCLCPP_INFO(this->get_logger(), "Goal frame is: %s!",goal_.header.frame_id.c_str());

        goal_publisher_->publish(goal_);
    }
    void clock_callback(const rosgraph_msgs::msg::Clock &msg)
    {
        sim_clock_ = msg;
        clock_update_ = true;
    }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoalPublisher>());
  rclcpp::shutdown();
  return 0;
}