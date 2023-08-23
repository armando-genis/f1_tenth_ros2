#ifndef LINEAR_INTERPOLATION_H
#define LINEAR_INTERPOLATION_H

// ---------------------------------------------------------------------
// 
// ---------------------------------------------------------------------

std::vector<Eigen::VectorXd> interpolateWaypoints(
    const std::vector<double>& x, 
    const std::vector<double>& y, 
    double INTERP_DISTANCE_RES) 
{
    std::vector<Eigen::VectorXd> waypoints_np;

    // Populate waypoints_np using x and y vectors
    for (size_t i = 0; i < x.size(); i++) {
        Eigen::VectorXd waypoint(2);
        waypoint[0] = x[i];
        waypoint[1] = y[i];
        waypoints_np.push_back(waypoint);
    }

    // Compute distances between waypoints
    std::vector<double> wp_distance;
    for (size_t i = 1; i < waypoints_np.size(); i++) {
        Eigen::VectorXd diff = waypoints_np[i] - waypoints_np[i - 1];
        wp_distance.push_back(diff.norm());
    }
    wp_distance.push_back(0.0);

    // Linear interpolation
    std::vector<Eigen::VectorXd> wp_interp;


    for (size_t i = 0; i < waypoints_np.size() - 1; i++) {
        wp_interp.push_back(waypoints_np[i]);

        int num_pts_to_interp = static_cast<int>(std::floor(wp_distance[i] / INTERP_DISTANCE_RES)) - 1;
        Eigen::VectorXd wp_vector = waypoints_np[i + 1] - waypoints_np[i];
        Eigen::VectorXd wp_uvector = wp_vector.normalized();

        for (int j = 0; j < num_pts_to_interp; j++) {
            Eigen::VectorXd next_wp_vector = INTERP_DISTANCE_RES * static_cast<double>(j + 1) * wp_uvector;
            wp_interp.push_back(waypoints_np[i] + next_wp_vector);
        }
    }
    wp_interp.push_back(waypoints_np.back());

    return wp_interp;
}

#endif
