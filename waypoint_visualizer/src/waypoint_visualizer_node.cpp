#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

class WaypointVisualizer : public rclcpp::Node
{
public:
    WaypointVisualizer() : Node("waypoint_visualizer")
    {

        this->declare_parameter("x", std::vector<double>{});
        this->declare_parameter("y", std::vector<double>{});

        this->get_parameter("x", x_waypoints_);
        this->get_parameter("y", y_waypoints_);

        if (!x_waypoints_.empty()) {
            RCLCPP_INFO(this->get_logger(), "First waypoint of x: %f", x_waypoints_[0]);
        } else {
            RCLCPP_WARN(this->get_logger(), "x_waypoints_ is empty!");
        }

        publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&WaypointVisualizer::publish_waypoints, this));
    }

private:
    void publish_waypoints()
    {
        visualization_msgs::msg::MarkerArray marker_array;

        for (size_t i = 0; i < x_waypoints_.size(); i++)
        {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "odom_demo";
            marker.header.stamp = this->get_clock()->now();
            marker.ns = "waypoints";
            marker.id = i;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = x_waypoints_[i];
            marker.pose.position.y = y_waypoints_[i];
            marker.pose.position.z = 0.0;
            marker.scale.x = 0.2;
            marker.scale.y = 0.2;
            marker.scale.z = 0.2;
            marker.color.a = 1.0; 
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;

            marker_array.markers.push_back(marker);
        }

        publisher_->publish(marker_array);
    }

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<double> x_waypoints_;
    std::vector<double> y_waypoints_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaypointVisualizer>());
    rclcpp::shutdown();
    return 0;
}
