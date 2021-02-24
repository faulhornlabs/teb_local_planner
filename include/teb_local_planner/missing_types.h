#pragma once

#include <Eigen/Core>

namespace teb_local_planner
{

struct Twist
{
    Eigen::Vector3d linear = Eigen::Vector3d::Zero();
    Eigen::Vector3d angular = Eigen::Vector3d::Zero();
};

struct Costmap2D
{

};

}
