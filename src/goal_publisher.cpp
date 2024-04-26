#include <chrono>
#include <functional>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

using namespace std::chrono_literals;

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

    goal_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal", 10);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&GoalPublisher::timer_callback, this));
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_publisher_;
    void timer_callback()
    {
        std::string goal_frame = this->get_parameter("goal_frame").as_string();
        RCLCPP_INFO(this->get_logger(), "Goal frame is: %s!", goal_frame.c_str());
    }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoalPublisher>());
  rclcpp::shutdown();
  return 0;
}