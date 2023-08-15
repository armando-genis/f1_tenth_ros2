#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <cmath>
#include <vector>

class PurePursuit : public rclcpp::Node {
public:
    PurePursuit() : Node("pure_pursuit_node") {

        this->declare_parameter("x", std::vector<double>{});
        this->declare_parameter("y", std::vector<double>{});

        this->get_parameter("x", x_waypoints_);
        this->get_parameter("y", y_waypoints_);

        // Subscriber to Odometry message
        subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("/racecar/odom_racecar", 10, std::bind(&PurePursuit::odom_callback, this, std::placeholders::_1));

        // Publisher for nearest waypoint
        waypoint_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("nearest_waypoint", 10);
        
        // Publisher for "hi_topic"
        hi_publisher_ = this->create_publisher<std_msgs::msg::String>("hi_topic", 10);
        
        timer_ = this->create_wall_timer(std::chrono::milliseconds(200), std::bind(&PurePursuit::pub_callback, this));

        RCLCPP_INFO(this->get_logger(), "pure_pursuit_node initialized");
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {

        int nearest_point_index = find_nearest_point(msg->pose.pose.position.x, msg->pose.pose.position.y);

        current_pose_x_ = msg->pose.pose.position.x;
        current_pose_y_ = msg->pose.pose.position.y;


        // RCLCPP_INFO(this->get_logger(), "Received Pose - x: %f, y: %f", current_pose_x_, current_pose_y_);


        if (nearest_point_index != -1) {
            // RCLCPP_INFO(this->get_logger(), "Nearest waypoint index: %d", nearest_point_index);
        

            geometry_msgs::msg::PoseStamped waypoint_msg;
            waypoint_msg.header.stamp = this->now();
            waypoint_msg.header.frame_id = "odom_demo";  // Assuming map frame, modify accordingly
            waypoint_msg.pose.position.x = x_waypoints_[nearest_point_index];
            waypoint_msg.pose.position.y = y_waypoints_[nearest_point_index];
            waypoint_publisher_->publish(waypoint_msg);
        }
    }

    void pub_callback() {
        std_msgs::msg::String msg;
        msg.data = "hi";
        hi_publisher_->publish(msg);
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
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr hi_publisher_;
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
