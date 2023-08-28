#include <rclcpp/rclcpp.hpp>
// ROS
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
// C++
#include <cmath>
#include <algorithm>
#include <Eigen/Dense>
#include <iostream>

#include <cstdio>
#include <vector>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "Linear_interpolation.h"
#include "subset_waypoints.h"


class StanleyController : public rclcpp::Node {
public:
    StanleyController() : Node("stanley_node")
     {

        this->declare_parameter("x", std::vector<double>{});
        this->declare_parameter("y", std::vector<double>{});
        this->declare_parameter("control_gain", 0.0);
        this->declare_parameter("softening_gain", 0.0);
        this->declare_parameter("yaw_rate_gain", 0.0);
        this->declare_parameter("steering_damp_gain", 0.0);
        this->declare_parameter("max_steer", 0.0);
        this->declare_parameter("wheelbase", 0.0);
        this->declare_parameter("dt", 0.0);

        this->get_parameter("x", x_waypoints_);
        this->get_parameter("y", y_waypoints_);
        this->get_parameter("control_gain", control_gain);
        this->get_parameter("softening_gain", softening_gain);
        this->get_parameter("yaw_rate_gain", yaw_rate_gain);
        this->get_parameter("steering_damp_gain", yaw_rate_gain);
        this->get_parameter("max_steer", yaw_rate_gain);
        this->get_parameter("wheelbase", yaw_rate_gain);
        this->get_parameter("dt", yaw_rate_gain);

        // Interpolate waypoints
        WaypointData data = interpolateWaypoints(x_waypoints_, y_waypoints_, 0.01);
        auto& interpolatedWaypoints = data.wp_interp;
        auto& hashWaypoints = data.wp_interp_hash;
        auto& wp_distance = data.wp_distance;
        RCLCPP_INFO(this->get_logger(), "value of dudoso : %zu", interpolatedWaypoints.size());

        for (size_t i = 0; i < x_waypoints_.size(); ++i) {
            waypoints_np.push_back({x_waypoints_[i], y_waypoints_[i]});
        }

        // Subscriber to Odometry message
        subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&StanleyController::odom_callback, this, std::placeholders::_1));

        // Publisher for "cmd"
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        next_marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("next_waypoint_marker", 10);
        // Timer for publishing "cmd"
        timer_ = this->create_wall_timer(std::chrono::milliseconds(200), std::bind(&StanleyController::pub_callback, this));

        RCLCPP_INFO(this->get_logger(), "pure_pursuit_node initialized");
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_pose_x_ = msg->pose.pose.position.x;
        current_pose_y_ = msg->pose.pose.position.y;  
        v = msg->twist.twist.linear.x;
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

        vehicle_yaw = yaw; 
        double degrees = vehicle_yaw * (180.0 / M_PI);
        // RCLCPP_INFO(this->get_logger(), "value of dudoso : %f", degrees);
    }

    void pub_callback() {
        // int target_idx = find_nearest_point(current_pose_x_,current_pose_y_);
        const auto& result = findClosestWaypoint(waypoints_np, current_pose_x_, current_pose_y_, wp_distance, hashWaypoints,interpolatedWaypoints);
        auto& closestWaypointIndex = result.closest_index;
        auto& newWaypoints = result.new_waypoints;

        marker_visualization(closestWaypointIndex);
    }

    void marker_visualization(int next_point_index){
         // Marker for next_point_index
        visualization_msgs::msg::Marker next_marker;
        next_marker.header.stamp = this->now();
        next_marker.header.frame_id = "odom";
        next_marker.type = visualization_msgs::msg::Marker::SPHERE;
        next_marker.action = visualization_msgs::msg::Marker::ADD;
        next_marker.pose.position.x = x_waypoints_[next_point_index];
        next_marker.pose.position.y = y_waypoints_[next_point_index];
        next_marker.scale.x = 0.2;
        next_marker.scale.y = 0.4;
        next_marker.scale.z = 0.4;
        next_marker.color.a = 1.0;
        next_marker.color.r = 1.0;   // blue color for differentiation
        next_marker.color.g = 0.0;
        next_marker.color.b = 0.0;
        
        next_marker_publisher_->publish(next_marker);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr next_marker_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    // variables of the parameters
    std::vector<double> x_waypoints_;
    std::vector<double> y_waypoints_;
    std::vector<std::vector<double>> waypoints_np;
    std::vector<Eigen::VectorXd> interpolatedWaypoints;
    std::vector<int> hashWaypoints;
    std::vector<double> wp_distance;
    double current_pose_x_= 0;
    double current_pose_y_= 0;
    double v = 0;

    double control_gain;
    double softening_gain;
    double yaw_rate_gain;
    double steering_damp_gain;
    double max_steer;
    double wheelbase;
    double dt;
    double vehicle_yaw;

};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StanleyController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
