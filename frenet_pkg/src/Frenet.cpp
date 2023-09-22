#include <rclcpp/rclcpp.hpp>
// ROS
#include <visualization_msgs/msg/marker_array.hpp>
// C++
#include <iostream>
#include <vector>
#include <Eigen/Dense>

using namespace std;

class Frenet : public rclcpp::Node
{
private:
    /* data */
    void timer_callback();
    void marker_array_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg);
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_subscriber_;

public:
    Frenet(/* args */);
    ~Frenet();
};

Frenet::Frenet(/* args */) : Node("Frenet_node")
{
    timer_ = this->create_wall_timer(200ms, std::bind(&Frenet::timer_callback, this));
    marker_array_subscriber_ = this->create_subscription<visualization_msgs::msg::MarkerArray>("/cluster_markers", 10, std::bind(&Frenet::marker_array_callback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Stanley_controller_node initialized");
}

Frenet::~Frenet()
{
}

void Frenet::timer_callback()
{
    RCLCPP_INFO(this->get_logger(), "Hello, world!");
}

void Frenet::marker_array_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
{
    // Handle the received message here
    RCLCPP_INFO(this->get_logger(), "Received a MarkerArray message!");
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Frenet>());
    rclcpp::shutdown();
    return 0;
}