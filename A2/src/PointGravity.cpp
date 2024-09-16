//
// Created by Karthik Iyer on 15/09/24.
//

#include "PointGravity.h"

#include <utility>

PointGravity::PointGravity(double mass, Eigen::Vector3d pos) : mass(mass), pos(std::move(pos)) {

}

Eigen::Vector3d PointGravity::getForce(Eigen::Vector3d &loc) const {
    Eigen::Vector3d dir = pos - loc;
    double r = dir.norm();
    dir /= r;
    return ((1/(r*r)) * (G * mass) * dir);
}