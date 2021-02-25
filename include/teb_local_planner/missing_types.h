#pragma once

#include <Eigen/Core>
#include <chrono>
#include <vector>

namespace teb_local_planner
{

struct Twist
{
    Eigen::Vector3d linear = Eigen::Vector3d::Zero();
    Eigen::Vector3d angular = Eigen::Vector3d::Zero();
};

struct Costmap2D
{
    double footprintCost(double x, double y, double theta, const std::vector< Eigen::Vector3d>& footprint_spec, double inscribed_radius = 0.0, double circumscribed_radius = 0.0)
    {
        return 0.0;
    }
};


struct TrajectoryPointMsg
{
    Eigen::Matrix4f pose;

    // Corresponding velocity
    Twist velocity;

    // Corresponding acceleration
    Twist acceleration;

    std::chrono::duration<double> time_from_start;
};

}
