#include <rclcpp/rclcpp.hpp>
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include <rosgraph_msgs/msg/clock.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;

class FootprintPublisher : public rclcpp::Node
{
public:
    FootprintPublisher()
    : rclcpp::Node("footprint_publisher")
    {
        this->declare_parameter("polygon_footprint.footprint_y", rclcpp::PARAMETER_DOUBLE_ARRAY);
        this->declare_parameter("polygon_footprint.frame_id", rclcpp::PARAMETER_STRING);
        this->declare_parameter("polygon_footprint.footprint_x", rclcpp::PARAMETER_DOUBLE_ARRAY);
        this->declare_parameter("sim_time", false);

        /*For using simulation time in gazebo*/
        use_sim_time_ = this->get_parameter("sim_time").as_bool();
        if(use_sim_time_)
        {
            clock_sub_ = this->create_subscription<rosgraph_msgs::msg::Clock>("/clock", rclcpp::SensorDataQoS(),
                            std::bind(&FootprintPublisher::clock_callback, this, _1));
        }

        use_circular_footprint = this->get_parameter("circular_footprint.use_circular_footprint").as_bool();
        footprint_pub_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>("/footprint", 10);
        timer_ = this->create_wall_timer(
        500ms, std::bind(&FootprintPublisher::timer_callback, this));
    }
private:
    /*For using simulation time in Gazebo*/
    rclcpp::Subscription<rosgraph_msgs::msg::Clock> ::SharedPtr clock_sub_;
    rosgraph_msgs::msg::Clock sim_clock_;
    bool use_sim_time_;
    bool clock_update_{false};

    geometry_msgs::msg::PolygonStamped footprint_;
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr footprint_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool use_circular_footprint;

    void timer_callback()
    {
        if(use_sim_time_)
        {
            footprint_.header.stamp.sec = sim_clock_.clock.sec;
            footprint_.header.stamp.nanosec = sim_clock_.clock.nanosec;
        }
        else
            footprint_.header.stamp = this->get_clock()->now();

        std::vector<double> footprint_y = this->get_parameter("polygon_footprint.footprint_y").as_double_array();
        std::vector<double> footprint_x = this->get_parameter("polygon_footprint.footprint_x").as_double_array();
        if (footprint_x.size() != footprint_y.size())
        { 
            RCLCPP_ERROR(this->get_logger(),"Inappropriate footprint");
            exit(1);
        }
        footprint_.header.frame_id = this->get_parameter("polygon_footprint.frame_id").as_string();
        for (std::size_t i{0}; i < footprint_x.size(); ++i)
        {
            geometry_msgs::msg::Point32 p;
            p.x = footprint_x[i];
            p.y = footprint_y[i];
            footprint_.polygon.points.push_back(p);
        }
        footprint_pub_->publish(footprint_);
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
    rclcpp::spin(std::make_shared<FootprintPublisher>());
    rclcpp::shutdown();
    return 0;
}