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
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>


// C++
#include <iostream>
#include <vector>
#include <Eigen/Dense>

// Linear Interpolation
#include "Linear_Interpolation.h"
//Stanley Controller
#include "StanleyController.h"

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

    // Stanley controller
    vector<Eigen::VectorXd> new_waypoints;

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void pub_callback();
    void marker_visualization(size_t next_point_index);
    void visualizeNewWaypoints();
    void publishAckermannDrive(double speed, double steering_angle);


    // ROS2
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_cmd_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr next_marker_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_newWaypoints_publisher_;
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
    ackermann_cmd_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/ackermann_cmd", 10);

    // publishers for visualization and debugging
    next_marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("next_waypoint_marker", 10);
    marker_newWaypoints_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("newWaypoints", 10);

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
    // RCLCPP_INFO(this->get_logger(), "Yaw in degrees: %f", degrees);

}

void StanleyNode::pub_callback()
{
    StanleyController controller(waypoints);
    controller.findClosestWaypoint(current_pose_x_, current_pose_y_, wp_distance, wp_interp_hash, wp_interp);
    size_t ClosestIndex = controller.getClosestIndex();
    marker_visualization(ClosestIndex);
    visualizeNewWaypoints();
    new_waypoints = controller.getNewWaypoints();
    controller.computeCrossTrackError(current_pose_x_, current_pose_y_, current_pose_yaw_);
    double target_idx = controller.GetTargetIdx();
    controller.computeSteeringAngle(current_pose_yaw_, current_velocity_);
    double steering_output = controller.GetDelta();
    RCLCPP_INFO(this->get_logger(), "steering_output: %f", steering_output);
    double computed_speed = 0.8;
    publishAckermannDrive(computed_speed, steering_output);
    // RCLCPP_INFO(this->get_logger(), "doing");
}

void StanleyNode::publishAckermannDrive(double speed, double steering_angle)
{
    ackermann_msgs::msg::AckermannDriveStamped drive_msg;
    drive_msg.header.stamp = this->now();
    drive_msg.header.frame_id = "base_link";
    drive_msg.drive.speed = speed;
    drive_msg.drive.steering_angle = steering_angle;
    ackermann_cmd_publisher_->publish(drive_msg);
}


// visualization of the next waypoint & new waypoints
void StanleyNode::marker_visualization(size_t next_point_index){
    // Marker for next_point_index
    visualization_msgs::msg::Marker next_marker;
    next_marker.header.stamp = this->now();
    next_marker.header.frame_id = "odom";
    next_marker.type = visualization_msgs::msg::Marker::CUBE;
    next_marker.action = visualization_msgs::msg::Marker::ADD;
    next_marker.pose.position.x = x_waypoints_[next_point_index];
    next_marker.pose.position.y = y_waypoints_[next_point_index];
    next_marker.scale.x = 0.15;
    next_marker.scale.y = 0.15;
    next_marker.scale.z = 0.15;
    next_marker.color.a = 1.0;
    next_marker.color.r = 1.0;   // blue color for differentiation
    next_marker.color.g = 0.8;
    next_marker.color.b = 0.0;
    
    next_marker_publisher_->publish(next_marker);
}

void StanleyNode::visualizeNewWaypoints() {
    visualization_msgs::msg::MarkerArray marker_array;

    for (size_t i = 0; i < new_waypoints.size(); ++i) {
        visualization_msgs::msg::Marker marker;
        marker.header.stamp = this->now();
        marker.header.frame_id = "odom";
        marker.id = i + 1000;  // Unique ID for each marker, added 1000 to distinguish from other markers
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = new_waypoints[i](0);  // Assuming x is the first element
        marker.pose.position.y = new_waypoints[i](1);  // Assuming y is the second element
        marker.scale.x = 0.12;  // Slightly larger than interpolated for differentiation
        marker.scale.y = 0.12;
        marker.scale.z = 0.12;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.5;
        
        marker_array.markers.push_back(marker);
    }
    
    marker_newWaypoints_publisher_->publish(marker_array);
}


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StanleyNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
