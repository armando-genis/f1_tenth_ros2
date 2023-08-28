#ifndef SUBSET_WAYPOINTS_H
#define SUBSET_WAYPOINTS_H

// ---------------------------------------------------------------------
// To reduce the amount of waypoints sent to the controller,
// provide a subset of waypoints that are within some 
// lookahead distance from the closest point to the car.
// ---------------------------------------------------------------------


struct ClosestWaypointResult {
    size_t closest_index;
    std::vector<Eigen::VectorXd> new_waypoints;
};

// Function to compute Euclidean distance
double computeDistance(double x1, double y1, double x2, double y2) {
    return std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

// Function to find the closest waypoint index
ClosestWaypointResult findClosestWaypoint(const std::vector<std::vector<double>>& waypoints_np, double current_x, double current_y,const std::vector<double>& wp_distance, const std::vector<int>& wp_interp_hash, const std::vector<Eigen::VectorXd>& wp_interp) {
    size_t closest_index = 0;  
    double closest_distance = computeDistance(waypoints_np[closest_index][0], waypoints_np[closest_index][1], current_x, current_y);
    double new_distance = closest_distance;
    size_t new_index = closest_index;

    while (new_distance <= closest_distance) {
        closest_distance = new_distance;
        closest_index = new_index;
        new_index++;
        if (new_index >= waypoints_np.size()) {  
            break;
        }
        new_distance = computeDistance(waypoints_np[new_index][0], waypoints_np[new_index][1], current_x, current_y);
    }

    new_distance = closest_distance;
    new_index = closest_index;

    while (new_distance <= closest_distance) {
        closest_distance = new_distance;
        closest_index = new_index;
        if (new_index == 0) {  // if at the beginning of the waypoints
            break;
        }
        new_index--;
        new_distance = computeDistance(waypoints_np[new_index][0], waypoints_np[new_index][1], current_x, current_y);
    }
    
    // Once the closest index is found, return the path that has 1
    // waypoint behind and X waypoints ahead, where X is the index
    // that has a lookahead distance specified by 
    // INTERP_LOOKAHEAD_DISTANCE = 20

    size_t waypoint_subset_first_index = closest_index - 1;
    if (waypoint_subset_first_index < 1) {
        waypoint_subset_first_index = 0;
    }

    size_t waypoint_subset_last_index = closest_index;
    double total_distance_ahead = 0.0;
    while (total_distance_ahead < 20) {
        total_distance_ahead += wp_distance[waypoint_subset_last_index];
        waypoint_subset_last_index++;

        if (waypoint_subset_last_index >= waypoints_np.size()) {
            waypoint_subset_last_index = waypoints_np.size() - 1;
            break;
        }
    }
    
    std::vector<Eigen::VectorXd> new_waypoints( 
        wp_interp.begin() + wp_interp_hash[waypoint_subset_first_index],
        wp_interp.begin() + wp_interp_hash[waypoint_subset_last_index] + 1
    );

    ClosestWaypointResult result;
    result.closest_index = closest_index;
    result.new_waypoints = new_waypoints;
    return result;

}

#endif