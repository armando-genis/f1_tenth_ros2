#include <rclcpp/rclcpp.hpp>
// ROS
#include <visualization_msgs/msg/marker_array.hpp>
#include <msg_custom_f1/msg/obstacle_data.hpp>
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
    void obstacle_arrays_callback(const msg_custom_f1::msg::ObstacleData::SharedPtr msg);
    void newWaypoints_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg);
    vector<uint32_t> cluster_sizes;
    vector<float> avg_distances;
    vector<vector<geometry_msgs::msg::Point>> cluster_points;
    vector<geometry_msgs::msg::Point> extracted_points;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<msg_custom_f1::msg::ObstacleData>::SharedPtr obstacle_subscriber_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr newWaypoints_subscriber_;

public:
    Frenet(/* args */);
    ~Frenet();
};

Frenet::Frenet(/* args */) : Node("Frenet_node")
{
    timer_ = this->create_wall_timer(200ms, std::bind(&Frenet::timer_callback, this));
    obstacle_subscriber_ = this->create_subscription<msg_custom_f1::msg::ObstacleData>("/cluster_markers", 10, std::bind(&Frenet::obstacle_arrays_callback, this, std::placeholders::_1));
    newWaypoints_subscriber_ = this->create_subscription<visualization_msgs::msg::MarkerArray>("newWaypoints", 10, std::bind(&Frenet::newWaypoints_callback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Stanley_controller_node initialized");
}

Frenet::~Frenet()
{
}

void Frenet::timer_callback()
{
    // RCLCPP_INFO(this->get_logger(), "Hello, world!");
}

void Frenet::obstacle_arrays_callback(const msg_custom_f1::msg::ObstacleData::SharedPtr msg)
{
    // Handle the received message here
    if (!msg->cluster_sizes.empty()) {
        RCLCPP_INFO(this->get_logger(), "Received a MarkerArray message!");
        cluster_sizes = msg->cluster_sizes;
        avg_distances = msg->avg_distances;

        cluster_points.clear();
        for (const auto& point_array : msg->cluster_points) {
            cluster_points.push_back(point_array.points);
        }
    }
    
}


void Frenet::newWaypoints_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
{
    // Handle the received message here
    RCLCPP_INFO(this->get_logger(), "Received a newWaypoints message!");
    extracted_points.clear();
    for (const auto& marker : msg->markers) {
        geometry_msgs::msg::Point point;
        point.x = marker.pose.position.x;
        point.y = marker.pose.position.y;
        point.z = marker.pose.position.z;
        extracted_points.push_back(point);
    }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Frenet>());
    rclcpp::shutdown();
    return 0;
}