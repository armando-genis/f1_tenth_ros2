#include <rclcpp/rclcpp.hpp>
// ROS
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// C++
#include <iostream>
#include <vector>
#include <Eigen/Dense>

// Linear Interpolation
#include "Linear_Interpolation.h"

using namespace std;

class StanleyNode : public rclcpp::Node
{
private:
    // Variables
    std::vector<double> x_waypoints_;
    std::vector<double> y_waypoints_;
    double current_pose_x_= 0;
    double current_pose_y_= 0;
    double current_pose_yaw_= 0;
    double current_velocity_ = 0;

    // linear interpolation
    vector<Eigen::VectorXd> waypoints;
    vector<Eigen::VectorXd> wp_interp;
    vector<int> wp_interp_hash;
    vector<double> wp_distance;



    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void pub_callback();

    // ROS2
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;


public:
    StanleyNode(/* args */);
    ~StanleyNode();
};


StanleyNode::StanleyNode(/* args */) : Node("stanley_node")
{
    this->declare_parameter("x", std::vector<double>{});
    this->declare_parameter("y", std::vector<double>{});
    this->get_parameter("x", x_waypoints_);
    this->get_parameter("y", y_waypoints_);

    // waypoints interpolation
    Linear_Interpolation linear_interpolation(x_waypoints_, y_waypoints_,0.1);
    linear_interpolation.interpolateWaypoints();
    waypoints = linear_interpolation.getWaypoints();
    wp_interp = linear_interpolation.getWp_interp();
    wp_interp_hash = linear_interpolation.getWp_interp_hash();
    wp_distance = linear_interpolation.getWp_distance();

    RCLCPP_INFO(this->get_logger(), "Waypoints interpolated");
    RCLCPP_INFO(this->get_logger(), "Waypoints size: %zu", waypoints.size());
    RCLCPP_INFO(this->get_logger(), "wp_interp size : %zu", wp_interp.size());

    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&StanleyNode::odom_callback, this, std::placeholders::_1));
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(200), std::bind(&StanleyNode::pub_callback, this));
    RCLCPP_INFO(this->get_logger(), "Stanley_controller_node initialized");
}

StanleyNode::~StanleyNode()
{
}


void StanleyNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    current_pose_x_ = msg->pose.pose.position.x;
    current_pose_y_ = msg->pose.pose.position.y;
    current_velocity_ = msg->twist.twist.linear.x;

    // Convert ROS2 Quaternion message to tf2 Quaternion
    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);

    // Extract yaw from the quaternion
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    current_pose_yaw_ = yaw;
    double degrees = current_pose_yaw_ * (180.0 / M_PI);
    RCLCPP_INFO(this->get_logger(), "Yaw in degrees: %f", degrees);

}


void StanleyNode::pub_callback()
{
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.5;
    cmd.angular.z = 0.5;
    cmd_vel_publisher_->publish(cmd);
}


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StanleyNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
