//
// Created by Karthik Iyer on 02/12/23.
//

#include "Gravity.h"

Gravity::Gravity(Eigen::Vector3d grav) : grav(grav) {

}

Eigen::Vector3d Gravity::getForce(Eigen::Vector3d &loc) const {
    return grav;
}

void Gravity::setGravity(Eigen::Vector3d grav) {
    this->grav = grav;
}