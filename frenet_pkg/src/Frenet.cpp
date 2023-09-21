#include <rclcpp/rclcpp.hpp>
// ROS


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
    rclcpp::TimerBase::SharedPtr timer_;
public:
    Frenet(/* args */);
    ~Frenet();
};

Frenet::Frenet(/* args */) : Node("Frenet_node")
{
    timer_ = this->create_wall_timer(200ms, std::bind(&Frenet::timer_callback, this));
    RCLCPP_INFO(this->get_logger(), "Stanley_controller_node initialized");
}

Frenet::~Frenet()
{
}

void Frenet::timer_callback()
{
    RCLCPP_INFO(this->get_logger(), "Hello, world!");
}


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Frenet>());
    rclcpp::shutdown();
    return 0;
}