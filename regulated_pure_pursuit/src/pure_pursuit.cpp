#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <cmath>
#include <vector>

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

        // Publisher for nearest waypoint
        waypoint_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("nearest_waypoint", 10);
        
        // Publisher for "hi_topic"

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

        if (nearest_point_index != -1) {
            geometry_msgs::msg::PoseStamped waypoint_msg;
            waypoint_msg.header.stamp = this->now();
            waypoint_msg.header.frame_id = "odom_demo";
            waypoint_msg.pose.position.x = x_waypoints_[nearest_point_index];
            waypoint_msg.pose.position.y = y_waypoints_[nearest_point_index];
            waypoint_publisher_->publish(waypoint_msg);

            // Calculate lookahead point
            double look_ahead_dist = MAX_VELOCITY * 0.5;
            int look_ahead_index = nearest_point_index;
            double cumulative_dist = 0.0;

            while(look_ahead_index < static_cast<int>(x_waypoints_.size()) - 1) {
                double dx = x_waypoints_[look_ahead_index+1] - x_waypoints_[look_ahead_index];
                double dy = y_waypoints_[look_ahead_index+1] - y_waypoints_[look_ahead_index];
                cumulative_dist += sqrt(dx*dx + dy*dy);
                
                if (cumulative_dist > look_ahead_dist) {
                    break;
                }
                look_ahead_index++;
            }

            double look_ahead_x = x_waypoints_[look_ahead_index];
            double look_ahead_y = y_waypoints_[look_ahead_index];

            // Compute steering angle
            double alpha = atan2(look_ahead_y - current_pose_y_, look_ahead_x - current_pose_x_);
            double ld2 = pow(look_ahead_x - current_pose_x_, 2) + pow(look_ahead_y - current_pose_y_, 2);
            double steering_angle = atan2(2.0 * WHEELBASE * sin(alpha), sqrt(ld2));
            
            // Clip steering angle
            if (steering_angle > MAX_STEERING_ANGLE) {
                steering_angle = MAX_STEERING_ANGLE;
            } else if (steering_angle < -MAX_STEERING_ANGLE) {
                steering_angle = -MAX_STEERING_ANGLE;
            }

            // Publish velocity and steering angle
            geometry_msgs::msg::Twist cmd_msg;
            cmd_msg.linear.x = MAX_VELOCITY;
            cmd_msg.angular.z = steering_angle;
            cmd_vel_publisher_->publish(cmd_msg);
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
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<double> x_waypoints_;
    std::vector<double> y_waypoints_;
    double current_pose_x_;
    double current_pose_y_;
    const double MAX_VELOCITY = 0.5;
    const double MAX_STEERING_ANGLE = 0.5;
    const double WHEELBASE = 0.35;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PurePursuit>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
