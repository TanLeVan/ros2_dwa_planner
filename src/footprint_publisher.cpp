#include <rclcpp/rclcpp.hpp>
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/polygon_stamped.hpp"

using namespace std::chrono_literals;

class FootprintPublisher : public rclcpp::Node
{
public:
    FootprintPublisher()
    : rclcpp::Node("footprint_publisher")
    {
        this->declare_parameter("circular_footprint.use_circular_footprint", rclcpp::PARAMETER_BOOL);
        this->declare_parameter("circular_footprint.radius", rclcpp::PARAMETER_DOUBLE);
        this->declare_parameter("polygon_footprint.footprint_x", rclcpp::PARAMETER_DOUBLE_ARRAY);
        this->declare_parameter("polygon_footprint.footprint_y", rclcpp::PARAMETER_DOUBLE_ARRAY);

        footprint_pub_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>("/footprint", 10);
        timer_ = this->create_wall_timer(
        500ms, std::bind(&FootprintPublisher::timer_callback, this));

        use_circular_footprint = this->get_parameter("circular_footprint.use_circular_footprint").as_bool();
    }
private:
    void timer_callback()
    {
        if(!use_circular_footprint){
            std::vector<double> footprint_x = this->get_parameter("polygon_footprint.footprint_x").as_double_array();
            std::vector<double> footprint_y = this->get_parameter("polygon_footprint.footprint_y").as_double_array();
            if (footprint_x.size() != footprint_y.size())
            { 
                RCLCPP_ERROR(this->get_logger(),"Inappropriate footprint");
                exit(1);
            }
            for (std::size_t i{0}; i < footprint_x.size(); ++i)
            {
                geometry_msgs::msg::Point32 p;
                p.x = footprint_x[i];
                p.y = footprint_y[i];
                footprint_.polygon.points.push_back(p);
            }
        }
        footprint_pub_->publish(footprint_);
    }
    geometry_msgs::msg::PolygonStamped footprint_;
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr footprint_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool use_circular_footprint{false};
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FootprintPublisher>());
    rclcpp::shutdown();
    return 0;
}