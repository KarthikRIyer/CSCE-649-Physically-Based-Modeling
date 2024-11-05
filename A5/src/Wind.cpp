//
// Created by Karthik Iyer on 02/12/23.
//

#include "Wind.h"

Wind::Wind(double strength, Eigen::Vector3d dir) : strength(strength), dir(dir) {
}

Eigen::Vector3d Wind::getForce(Eigen::Vector3d &loc) const {
    return dir.normalized() * strength;
}

void Wind::setStrength(double strength) {
    this->strength = strength;
}

void Wind::setDirection(Eigen::Vector3d dir) {
    this->dir = dir;
}