#include "Linear_Interpolation.h"

Linear_Interpolation::Linear_Interpolation(const std::vector<double>& x_vals, const std::vector<double>& y_vals, double interp_distance_res) : INTERP_DISTANCE_RES(interp_distance_res)
{
    std::cout << "Linear_Interpolation Constructor called" << std::endl;
    if (x_vals.size() != y_vals.size()) {
        throw std::runtime_error("Size mismatch between x and y vectors");
    }

    for (size_t i = 0; i < x_vals.size(); i++) {
        Eigen::VectorXd waypoint(2);
        waypoint[0] = x_vals[i];
        waypoint[1] = y_vals[i];
        waypoints.push_back(waypoint);
    }
}

Linear_Interpolation::~Linear_Interpolation()
{
    std::cout << "Linear_Interpolation Destructor called" << std::endl;
}   

void Linear_Interpolation::interpolateWaypoints() {
    for (size_t i = 1; i < waypoints.size(); i++) {
        Eigen::VectorXd diff = waypoints[i] - waypoints[i - 1];
        wp_distance.push_back(diff.norm());
    }
    wp_distance.push_back(0.0);

    int interp_counter = 0;

    for (size_t i = 0; i < waypoints.size() - 1; i++) {
        Eigen::VectorXd point_with_yaw(3);
        point_with_yaw << waypoints[i](0), waypoints[i](1), 0.0; // default yaw
        wp_interp.push_back(point_with_yaw);
        wp_interp_hash.push_back(interp_counter);
        interp_counter++;

        int num_pts_to_interp = static_cast<int>(std::floor(wp_distance[i] / INTERP_DISTANCE_RES)) - 1;
        Eigen::VectorXd wp_vector = waypoints[i + 1] - waypoints[i];
        Eigen::VectorXd wp_uvector = wp_vector.normalized();

        for (int j = 0; j < num_pts_to_interp; j++) {
            Eigen::VectorXd next_wp_vector = INTERP_DISTANCE_RES * static_cast<double>(j + 1) * wp_uvector;
            Eigen::VectorXd next_point(2);
            next_point = waypoints[i] + next_wp_vector;
            
            double dy = next_point(1) - waypoints[i](1);
            double dx = next_point(0) - waypoints[i](0);
            double yaw = std::atan2(dy, dx);
            
            Eigen::VectorXd next_point_with_yaw(3);
            next_point_with_yaw << next_point(0), next_point(1), yaw;

            wp_interp.push_back(next_point_with_yaw);
            interp_counter++;
        }
    }
    Eigen::VectorXd last_point_with_yaw(3);
    last_point_with_yaw << waypoints.back()(0), waypoints.back()(1), 0.0; // default yaw for last point
    wp_interp.push_back(last_point_with_yaw);
    wp_interp_hash.push_back(interp_counter);
}

double Linear_Interpolation::getSize() const{
    return wp_interp.size();
}

std::vector<Eigen::VectorXd> Linear_Interpolation::getWaypoints() const{
    // return the waypoints in a eigen vector
    return waypoints;
}

std::vector<Eigen::VectorXd> Linear_Interpolation::getWp_interp() const{
    // return the new waypoints 
    return wp_interp;
}

std::vector<int> Linear_Interpolation::getWp_interp_hash() const{
    return wp_interp_hash;
}

std::vector<double> Linear_Interpolation::getWp_distance() const{
    return wp_distance;
}