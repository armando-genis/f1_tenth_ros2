#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <cmath>
#include <vector>
#include <algorithm>


class PurePursuit : public rclcpp::Node {
public:
    PurePursuit() : Node("pure_pursuit_node")
     {

        this->declare_parameter("x", std::vector<double>{});
        this->declare_parameter("y", std::vector<double>{});

        this->get_parameter("x", x_waypoints_);
        this->get_parameter("y", y_waypoints_);

        // Subscriber to Odometry message
        subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("/racecar/odom_racecar", 10, std::bind(&PurePursuit::odom_callback, this, std::placeholders::_1));

        // Publisher for nearest waypoint and the next waypoint
        marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("nearest_waypoint_marker", 10);
        next_marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("next_waypoint_marker", 10);

        
        // Publisher for "cmd"
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/racecar/cmd_racecar", 10);
        
        timer_ = this->create_wall_timer(std::chrono::milliseconds(200), std::bind(&PurePursuit::pub_callback, this));

        RCLCPP_INFO(this->get_logger(), "pure_pursuit_node initialized");
    }

private:


    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_pose_x_ = msg->pose.pose.position.x;
        current_pose_y_ = msg->pose.pose.position.y;
    }

    void pub_callback() {

        int nearest_point_index = find_nearest_point(current_pose_x_, current_pose_y_);
        int next_point_index = nearest_point_index + 1;

        if (nearest_point_index != -1) {
            visualization_msgs::msg::Marker marker;
            marker.header.stamp = this->now();
            marker.header.frame_id = "odom_demo";
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = x_waypoints_[nearest_point_index];
            marker.pose.position.y = y_waypoints_[nearest_point_index];
            marker.pose.position.z = 0;
            marker.scale.x = 0.2; // Adjust as needed
            marker.scale.y = 0.2; // Adjust as needed
            marker.scale.z = 0.2; // Adjust as needed
            marker.color.a = 1.0; // Must be non-zero
            marker.color.r = 1.0; // Red color
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            
            marker_publisher_->publish(marker);

            // Check if next_point_index is valid
            if (next_point_index < static_cast<int>(x_waypoints_.size())) {
                // Marker for next_point_index
                visualization_msgs::msg::Marker next_marker;
                next_marker.header.stamp = this->now();
                next_marker.header.frame_id = "odom_demo";
                next_marker.type = visualization_msgs::msg::Marker::SPHERE;
                next_marker.action = visualization_msgs::msg::Marker::ADD;
                next_marker.pose.position.x = x_waypoints_[next_point_index];
                next_marker.pose.position.y = y_waypoints_[next_point_index];
                next_marker.scale.x = 0.2;
                next_marker.scale.y = 0.2;
                next_marker.scale.z = 0.2;
                next_marker.color.a = 1.0;
                next_marker.color.r = 0.0;   // blue color for differentiation
                next_marker.color.g = 0.0;
                next_marker.color.b = 1.0;
                
                next_marker_publisher_->publish(next_marker);
            }

        }
    }


    int find_nearest_point(double curr_x, double curr_y) {
        std::vector<double> ranges;
        for (size_t index = 0; index < x_waypoints_.size(); ++index) {
            double eucl_x = std::pow(curr_x - x_waypoints_[index], 2);
            double eucl_y = std::pow(curr_y - y_waypoints_[index], 2);
            double eucl_d = std::sqrt(eucl_x + eucl_y);
            ranges.push_back(eucl_d);
        }
        auto min_it = std::min_element(ranges.begin(), ranges.end());
        return std::distance(ranges.begin(), min_it);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr next_marker_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<double> x_waypoints_;
    std::vector<double> y_waypoints_;
    double current_pose_x_;
    double current_pose_y_;


};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PurePursuit>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
