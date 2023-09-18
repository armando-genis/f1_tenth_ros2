#include "StanleyController.h"

StanleyController::StanleyController(vector<Eigen::VectorXd> waypoints) : waypoints(waypoints)
{
    max_steer = GetAngleToRadians(40.0);
}

StanleyController::~StanleyController()
{

}

vector<Eigen::VectorXd> StanleyController::getNewWaypoints() const{
    return new_waypoints;
}

size_t StanleyController::getClosestIndex() const{
    return closest_index;
}

double StanleyController::GetMaxSteer() const{
    return max_steer;
}

double StanleyController::GetTargetIdx() const{
    return target_idx;
}

double StanleyController::GetErrorFrontAxle() const{
    return error_front_axle;
}

double StanleyController::GetPid() const{
    return pid;
}

double StanleyController::GetDelta() const{
    return delta;
}

double StanleyController::GetNormaliceAngle(double angle){
    // ---------------------------------------------------------------------
    // Normalize an angle to [-pi, pi].
    // :param angle: (double)
    // :return: (double) double in radian in [-pi, pi]
    // ---------------------------------------------------------------------
    if (angle > M_PI) angle -= 2 * M_PI;
    if (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}

double StanleyController::GetAngleToRadians(double angle_in_degrees){
    // ---------------------------------------------------------------------
    // Cover an angle in degrees to radians.
    // :param angle_in_degrees: (double) 
    // :return: (double) angle in radian
    // ---------------------------------------------------------------------
    double radians = angle_in_degrees * (M_PI / 180.0);
    return radians;
}

double StanleyController::computeDistance(double x1, double y1, double x2, double y2){
    // ---------------------------------------------------------------------
    // Compute the distance between two points.
    // :param x1: (double) x coordinate of point 1
    // :param y1: (double) y coordinate of point 1
    // :param x2: (double) x coordinate of point 2
    // :param y2: (double) y coordinate of point 2
    // :return: (double) distance between two points
    // ---------------------------------------------------------------------
    return std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

// ---------------------------------------------------------------------
// To reduce the amount of waypoints sent to the controller,
// provide a subset of waypoints that are within some 
// lookahead distance from the closest point to the car.
// ---------------------------------------------------------------------

void StanleyController::findClosestWaypoint(double current_x, double current_y,const vector<double>& wp_distance, const vector<int>& wp_interp_hash, const vector<Eigen::VectorXd>& wp_interp)
{
    // ---------------------------------------------------------------------
    // Find the closest waypoint to the car and To reduce the amount of waypoints sent to the controller, provide a subset of waypoints that are within some lookahead distance from the closest point to the car.
    // :param current_x: (double) current x position of the car
    // :param current_y: (double) current y position of the car
    // :param wp_distance: (vector<double>) distance between waypoints
    // :param wp_interp_hash: (vector<int>) hash table of waypoints
    // :param wp_interp: (vector<Eigen::VectorXd>) interpolated waypoints
    // :return: (int) index of the closest waypoint
    // :return: (vector<Eigen::VectorXd>) subset of waypoints
    // ---------------------------------------------------------------------

    // Compute distance to each waypoint
    vector<double> distances;
    for (size_t i = 0; i < waypoints.size(); i++) {
        distances.push_back(computeDistance(waypoints[i][0], waypoints[i][1], current_x, current_y));
    }

    // Find the index of the closest waypoint
    auto min_dist_it = min_element(distances.begin(), distances.end());
    closest_index = distance(distances.begin(), min_dist_it);

    // Once the closest index is found, return the path that has 1
    // waypoint behind and X waypoints ahead, where X is the index
    // that has a lookahead distance specified by 
    // INTERP_LOOKAHEAD_DISTANCE = 20

    size_t waypoint_subset_first_index = closest_index > 0 ? closest_index - 1 : 0;

    size_t waypoint_subset_last_index = closest_index;
    double total_distance_ahead = 0.0;

    while (total_distance_ahead < 3) {
        if (waypoint_subset_last_index >= waypoints.size() || waypoint_subset_last_index >= wp_distance.size()) {
            waypoint_subset_last_index = std::min(waypoints.size(), wp_distance.size()) - 1;
            break;
        }
        total_distance_ahead += wp_distance[waypoint_subset_last_index];
        waypoint_subset_last_index++;
    }

    vector<Eigen::VectorXd> new_waypoints_data( 
        wp_interp.begin() + wp_interp_hash[waypoint_subset_first_index],
        wp_interp.begin() + wp_interp_hash[waypoint_subset_last_index] + 1
    );

    new_waypoints = new_waypoints_data;
}


void StanleyController::computeCrossTrackError(double current_x, double current_y, double current_yaw){
    current_yaw = GetNormaliceAngle(current_yaw);
    double fx = current_x + L * cos(current_yaw);
    double fy = current_y + L * sin(current_yaw);
    // Search nearest point index

    // Search nearest point index

    double min_dist = std::numeric_limits<double>::max();

    for (size_t i = 0; i < new_waypoints.size(); i++) {
        double dx = fx - new_waypoints[i](0);  // Access x-coordinate of Eigen::VectorXd
        double dy = fy - new_waypoints[i](1);  // Access y-coordinate of Eigen::VectorXd
        double dist = std::sqrt(dx * dx + dy * dy);

        if (dist < min_dist) {
            min_dist = dist;
            target_idx = i;
        }
    }

    double front_axle_vec[2] = {-std::cos(current_yaw + M_PI / 2), -std::sin(current_yaw + M_PI / 2)};
    error_front_axle = (fx - new_waypoints[target_idx](0)) * front_axle_vec[0] + (fy - new_waypoints[target_idx](1)) * front_axle_vec[1];
}

void StanleyController::computePID(double target, double current){
    pid = Kp * (target - current);
}

void StanleyController::computeSteeringAngle(double current_yaw, double v){
    current_yaw = GetNormaliceAngle(current_yaw);
    // double yaw_target = std::atan2(new_waypoints[target_idx][1]-new_waypoints[target_idx-1][1], new_waypoints[target_idx][0]-new_waypoints[target_idx-1][0]);
    // double yaw_target2 = new_waypoints[target_idx](2);
    // yaw_target = GetNormaliceAngle(yaw_target);
    // yaw_target2 = GetNormaliceAngle(yaw_target2);
    // double yaw_of_the_last_point = std::atan2(new_waypoints.back()[1]-new_waypoints[new_waypoints.size()-2][1], new_waypoints.back()[0]-new_waypoints[new_waypoints.size()-2][0]);
    // yaw_of_the_last_point = GetNormaliceAngle(yaw_of_the_last_point);
    double yaw_path = std::atan2(new_waypoints.back()[1]-new_waypoints.front()[1], new_waypoints.back()[0]-new_waypoints.front()[0]);
    yaw_path = GetNormaliceAngle(yaw_path);

    double theta_e = GetNormaliceAngle(yaw_path - current_yaw);
    // theta_e corrects the heading error
    double theta_d = atan2(K * error_front_axle, v);
    delta = theta_e + theta_d;
    delta = GetNormaliceAngle(delta);

}



// void StanleyController::findClosestWaypoint(double current_x, double current_y,const vector<double>& wp_distance, const vector<int>& wp_interp_hash, const vector<Eigen::VectorXd>& wp_interp)
// {
//     // ---------------------------------------------------------------------
//     // Find the closest waypoint to the car and To reduce the amount of waypoints sent to the controller, provide a subset of waypoints that are within some lookahead distance from the closest point to the car.
//     // :param current_x: (double) current x position of the car
//     // :param current_y: (double) current y position of the car
//     // :param wp_distance: (vector<double>) distance between waypoints
//     // :param wp_interp_hash: (vector<int>) hash table of waypoints
//     // :param wp_interp: (vector<Eigen::VectorXd>) interpolated waypoints
//     // :return: (int) index of the closest waypoint
//     // :return: (vector<Eigen::VectorXd>) subset of waypoints
//     // ---------------------------------------------------------------------


//     double closest_distance = computeDistance(waypoints[closest_index][0], waypoints[closest_index][1], current_x, current_y);
//     double new_distance = closest_distance;
//     size_t new_index = closest_index;

//     while (new_distance <= closest_distance) {
//         closest_distance = new_distance;
//         closest_index = new_index;
//         new_index++;
//         if (new_index >= waypoints.size()) {  
//             break;
//         }
//         new_distance = computeDistance(waypoints[new_index][0], waypoints[new_index][1], current_x, current_y);
//     }

//     new_distance = closest_distance;
//     new_index = closest_index;

//     while (new_distance <= closest_distance) {
//         closest_distance = new_distance;
//         closest_index = new_index;
//         if (new_index == 0) {  // if at the beginning of the waypoints
//             break;
//         }
//         new_index--;
//         new_distance = computeDistance(waypoints[new_index][0], waypoints[new_index][1], current_x, current_y);
//     }
    
//     // Once the closest index is found, return the path that has 1
//     // waypoint behind and X waypoints ahead, where X is the index
//     // that has a lookahead distance specified by 
//     // INTERP_LOOKAHEAD_DISTANCE = 20

//     size_t waypoint_subset_first_index = closest_index - 1;
//     if (waypoint_subset_first_index < 1) {
//         waypoint_subset_first_index = 0;
//     }

//     size_t waypoint_subset_last_index = closest_index;
//     double total_distance_ahead = 0.0;

//     while (total_distance_ahead < 3) {
//         if (waypoint_subset_last_index >= waypoints.size() || waypoint_subset_last_index >= wp_distance.size()) {
//             waypoint_subset_last_index = std::min(waypoints.size(), wp_distance.size()) - 1;
//             break;
//         }
//         total_distance_ahead += wp_distance[waypoint_subset_last_index];
//         waypoint_subset_last_index++;
//     }

//     vector<Eigen::VectorXd> new_waypoints_data( 
//         wp_interp.begin() + wp_interp_hash[waypoint_subset_first_index],
//         wp_interp.begin() + wp_interp_hash[waypoint_subset_last_index] + 1
//     );

//     new_waypoints = new_waypoints_data;
// }
