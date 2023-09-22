#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <msg_custom_f1/msg/obstacle_data.hpp>


using namespace std;

class Obstacles : public rclcpp::Node
{
private:
    // variables
    float angle_min;
    float angle_max;
    float angle_increment;
    float range_min;
    float range_max;
    vector<float> ranges;
    vector<std::vector<geometry_msgs::msg::Point>> clusters_points;

    // function declarations
    void pub_callback();
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void clusters_points_data();


    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_publisher_;
    rclcpp::Publisher<msg_custom_f1::msg::ObstacleData>::SharedPtr obstacle_data_publisher_;

public:
    Obstacles(/* args */);
    ~Obstacles();
};

Obstacles::Obstacles(/* args */): Node("Obstacle_avoidance_node")
{

    timer_ = this->create_wall_timer(std::chrono::milliseconds(200), std::bind(&Obstacles::pub_callback, this));
    scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&Obstacles::scan_callback, this, std::placeholders::_1));
    marker_array_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/cluster_markers", 10);
    obstacle_data_publisher_ = this->create_publisher<msg_custom_f1::msg::ObstacleData>("/obstacle_data", 10);
    RCLCPP_INFO(this->get_logger(), "obstacle_avoidance_node initialized");

}

Obstacles::~Obstacles()
{
}

void Obstacles::pub_callback()
{

    // RCLCPP_INFO(this->get_logger(), "working");

}

void Obstacles::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    // Extracting LaserScan information:
    angle_min = msg->angle_min;
    angle_max = msg->angle_max;
    angle_increment = msg->angle_increment;
    range_min = msg->range_min;
    range_max = msg->range_max;
    ranges = msg->ranges;

    // http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Point.html
    // make a cluster of points when a obstacle is detected
    vector<std::vector<geometry_msgs::msg::Point>> clusters;
    vector<geometry_msgs::msg::Point> current_cluster;

    float x_prev = 0, y_prev = 0;
    bool prev_inf = true;
    float thresh_dist_points = 0.5;
    float thresh_dist = 0.9;
    int numOfClusters = 0;

    for (size_t i = 0; i < ranges.size(); ++i) {
        float distance = ranges[i];
        float theta = angle_min + (i * angle_increment);
        float x_curr = ranges[i] * cos(theta);
        float y_curr = ranges[i] * sin(theta);
        geometry_msgs::msg::Point p;
        p.x = x_curr;
        p.y = y_curr;
        p.z = 0.0;

        // a obstacle is detected if the distance is less than 0.9m
        if (distance < thresh_dist) {
            if (isfinite(ranges[i])) {
                // If the distance between the current point and the previous point is greater than 0.5m, then the current point is considered to be part of a new cluster
                if (prev_inf == false) {
                    float distance_to_prev_point = sqrt(pow((x_curr-x_prev),2) + pow((y_curr-y_prev),2));

                    if (distance_to_prev_point < thresh_dist_points) {
                        current_cluster.push_back(p);
                    } else {
                        if (!current_cluster.empty()) {
                            clusters.push_back(current_cluster);
                            current_cluster.clear();
                        }
                        current_cluster.push_back(p);
                        numOfClusters++;
                    }
                } else {
                    current_cluster.push_back(p);
                    if (current_cluster.size() == 1) {
                        numOfClusters++;
                    }
                }
                x_prev = x_curr;
                y_prev = y_curr;
                prev_inf = false;

            } else {
                if (!current_cluster.empty()) {
                    clusters.push_back(current_cluster);
                    current_cluster.clear();
                }
                prev_inf = true;
            }
        }
    }
    if (!current_cluster.empty()) {
        clusters.push_back(current_cluster);
    }

    // RCLCPP_INFO(this->get_logger(), "Number of clusters detected: %d", numOfClusters);
    clusters_points.resize(clusters.size());

    vector<uint32_t> cluster_sizes;
    vector<float> avg_distances;

    // Visualize the clusters and calculate the average distance of the points in the cluster
    visualization_msgs::msg::MarkerArray marker_array;
    
    for (size_t cluster_id = 0; cluster_id < clusters.size(); cluster_id++)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "base_link";
        marker.header.stamp = this->now();
        marker.ns = "clusters";
        marker.id = cluster_id;
        marker.type = visualization_msgs::msg::Marker::POINTS;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.color.r = static_cast<float>(rand()) / RAND_MAX; 
        marker.color.g = static_cast<float>(rand()) / RAND_MAX;
        marker.color.b = static_cast<float>(rand()) / RAND_MAX;
        marker.color.a = 1.0;
        marker.lifetime = rclcpp::Duration(0, 500000000); 

        // compute the size of the cluster. How many individual points (from the LaserScan) were grouped together to form that particular cluster.
        size_t cluster_size = clusters[cluster_id].size();
        cluster_sizes.push_back(static_cast<uint32_t>(cluster_size));
        float total_distance = 0.0f;

        // Add points of the cluster to the marker
        for (const auto &pt : clusters[cluster_id]) {
            marker.points.push_back(pt);
            clusters_points[cluster_id].push_back(pt);
            // compute the average distance of the points in the cluster
            float distance = sqrt(pt.x * pt.x + pt.y * pt.y);
            total_distance += distance;

        }
        avg_distances.push_back(total_distance / cluster_size);
        marker_array.markers.push_back(marker);
        
    }

    marker_array_publisher_->publish(marker_array);
    // for (size_t i = 0; i < clusters.size(); i++) {
    //     RCLCPP_INFO(this->get_logger(), "Cluster %zu: Size = %u, Avg Distance = %f", i, cluster_sizes[i], avg_distances[i]);
    // }

    // Populate and publish the custom message
    msg_custom_f1::msg::ObstacleData obstacle_msg;
    for (const auto &cluster : clusters_points) {
        msg_custom_f1::msg::PointArray point_array_msg;
        point_array_msg.points = cluster;
        obstacle_msg.cluster_points.push_back(point_array_msg);
    }
    obstacle_msg.cluster_sizes = cluster_sizes;
    obstacle_msg.avg_distances = avg_distances;
    obstacle_data_publisher_->publish(obstacle_msg);
}



int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Obstacles>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}