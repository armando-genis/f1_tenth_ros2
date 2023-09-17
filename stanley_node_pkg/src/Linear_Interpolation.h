#ifndef LINEAR_INTERPOLATION_H
#define LINEAR_INTERPOLATION_H
#include <iostream>
#include <vector>
#include <Eigen/Dense>

class Linear_Interpolation
{
private:
    std::vector<Eigen::VectorXd> waypoints;
    std::vector<Eigen::VectorXd> wp_interp;
    std::vector<int> wp_interp_hash;
    std::vector<double> wp_distance;
    double INTERP_DISTANCE_RES;
public:
    Linear_Interpolation(const std::vector<double>& x_vals, const std::vector<double>& y_vals, double interp_distance_res = 0.01);
    ~Linear_Interpolation();
    void interpolateWaypoints();
    double getSize() const;
    std::vector<Eigen::VectorXd> getWp_interp() const;
    std::vector<Eigen::VectorXd> getWaypoints() const;
    std::vector<int> getWp_interp_hash() const;
    std::vector<double> getWp_distance() const;
};


#endif
