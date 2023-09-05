#ifndef STANLEY_ALGORITHM_H
#define STANLEY_ALGORITHM_H


#include <vector>
#include <cmath>
#include <limits>

// ---------------------------------------------------------------------
// 
// ---------------------------------------------------------------------

double setSteer(double input_steer_in_rad);

double lateralController(const std::vector<Eigen::VectorXd>& waypoints, double x, double y, double yaw, double v) {
    // Initial setup
    double steer_output = 0;

    // Heading error
    double yaw_path = std::atan2(waypoints.back()[1]-waypoints.front()[1], waypoints.back()[0]-waypoints.front()[0]);
    double yaw_diff_heading = yaw_path - yaw;


    double crosstrack_error = std::numeric_limits<double>::max();
    for (const auto& waypoint : waypoints) {
        double dist = std::pow(x - waypoint[0], 2) + std::pow(y - waypoint[1], 2);
        crosstrack_error = std::min(crosstrack_error, dist);
    }


    
    return crosstrack_error;
}

double setSteer(double input_steer_in_rad) {
    static double _set_steer = 0.0;
    double _conv_rad_to_steer = 180.0 / 70.0 / M_PI;
    // Convert radians to [-1, 1]
    double input_steer = _conv_rad_to_steer * input_steer_in_rad;

    // Clamp the steering command to valid bounds
    _set_steer = std::max(std::min(input_steer, 1.0), -1.0);

    return _set_steer;
}


#endif



    // // Crosstrack error
    // double crosstrack_error = std::numeric_limits<double>::max();
    // for (const auto& waypoint : waypoints) {
    //     double dist = std::pow(x - waypoint[0], 2) + std::pow(y - waypoint[1], 2);
    //     crosstrack_error = std::min(crosstrack_error, dist);
    // }

    
    // crosstrack_error = std::sqrt(crosstrack_error);
    // double yaw_cross_track = std::atan2(y - waypoints.front()[1], x - waypoints.front()[0]);
    // double yaw_path2ct = yaw_path - yaw_cross_track;
    // if (yaw_path2ct > M_PI) yaw_path2ct -= 2 * M_PI;
    // if (yaw_path2ct < -M_PI) yaw_path2ct += 2 * M_PI;
    // if (yaw_path2ct > 0) crosstrack_error = std::abs(crosstrack_error);
    // else crosstrack_error = -std::abs(crosstrack_error);
    // double k_e = 0.3;
    // double yaw_diff_crosstrack = std::atan(k_e * crosstrack_error / v);

    // // Final expected steering
    // double steer_expect = yaw_diff_crosstrack + yaw_diff_heading;
    // if (steer_expect > M_PI) steer_expect -= 2 * M_PI;
    // if (steer_expect < -M_PI) steer_expect += 2 * M_PI;

    // steer_expect = std::min(1.22, steer_expect);
    // steer_expect = std::max(-1.22, steer_expect);

    // // Update
    
    // steer_output = setSteer(steer_expect);