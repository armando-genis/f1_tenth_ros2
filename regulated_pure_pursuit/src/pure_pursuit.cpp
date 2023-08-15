#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <vector>
#include <std_msgs/msg/string.hpp>

class PurePursuit : public rclcpp::Node {
public:
    PurePursuit() : Node("pure_pursuit_node") {

        this->declare_parameter("x", std::vector<double>{});
        this->declare_parameter("y", std::vector<double>{});

        // Read 'x' and 'y' parameter as a vector
        this->get_parameter("x", x_waypoints_);
        this->get_parameter("y", y_waypoints_);

        RCLCPP_INFO(this->get_logger(), "First waypoint of x: %f", x_waypoints_[0]);
        RCLCPP_INFO(this->get_logger(), "First waypoint of y: %f", y_waypoints_[0]);


        // publisher and timer
        publisher_ = this->create_publisher<std_msgs::msg::String>("hi_topic", 10);

        timer_ = this->create_wall_timer(
        std::chrono::milliseconds(200), std::bind(&PurePursuit::pub_callback, this));

        RCLCPP_INFO(this->get_logger(), "pure_pursuit_node initialized");
        
    }
private:

    void pub_callback() {
        std_msgs::msg::String msg;
        msg.data = "hi";
        // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg.data.c_str());

        publisher_->publish(msg);
    }


    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<double> x_waypoints_;
    std::vector<double> y_waypoints_;

};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PurePursuit>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}



