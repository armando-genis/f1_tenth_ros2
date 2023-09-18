#ifndef STANLEYCONTROLLER_H
#define STANLEYCONTROLLER_H
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <cmath>
using namespace std;


class StanleyController
{
private:
    vector<Eigen::VectorXd> waypoints; 
    double L = 0.40; // [m] Wheel base of vehicle
    double K = 0.5;  // control gain
    double Kp = 1.0; // speed proportional gain
    double max_steer = 0.0; // [rad] max steering angle
    size_t closest_index = 0.0; // Closaest waypoint index
    vector<Eigen::VectorXd> new_waypoints; //subset of waypoints
    double error_front_axle = 0.0;
    double target_idx = 0.0;
    double pid = 0.0;
    double delta = 0.0;

public:
    StanleyController(vector<Eigen::VectorXd> waypoints);
    ~StanleyController();
    // function to compute the main functions of the controller (Utility functions)
    double GetNormaliceAngle(double angle);
    double GetAngleToRadians(double angle_in_degrees);
    double computeDistance(double x1, double y1, double x2, double y2);
    // function to find the closest waypoint to the car
    void findClosestWaypoint(double current_x, double current_y,const vector<double>& wp_distance, const vector<int>& wp_interp_hash, const vector<Eigen::VectorXd>& wp_interp);
    // function to compute the cross track error and get the target of the new waypoints
    void computeCrossTrackError(double current_x, double current_y, double current_yaw);
    // function to compute PID
    void computePID(double target, double current);
    // function to compute the stanley controlle
    void computeSteeringAngle(double current_yaw, double v);

    // function to return data
    vector<Eigen::VectorXd> getNewWaypoints() const;
    size_t getClosestIndex() const;
    double GetMaxSteer() const;
    double GetTargetIdx() const;
    double GetErrorFrontAxle() const;
    double GetPid() const;
    double GetDelta() const;

    // void Controller();
};

#endif
