#include <rclcpp/rclcpp.hpp>
using namespace std;

class Obstacles : public rclcpp::Node
{
private:
    /* data */
    void pub_callback();
    rclcpp::TimerBase::SharedPtr timer_;
public:
    Obstacles(/* args */);
    ~Obstacles();
};

Obstacles::Obstacles(/* args */): Node("Obstacle_avoidance_node")
{

    timer_ = this->create_wall_timer(std::chrono::milliseconds(200), std::bind(&Obstacles::pub_callback, this));
    RCLCPP_INFO(this->get_logger(), "obstacle_avoidance_node initialized");

}

Obstacles::~Obstacles()
{
}


void Obstacles::pub_callback()
{

    RCLCPP_INFO(this->get_logger(), "working");

}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Obstacles>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}