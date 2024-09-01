//
// Created by Karthik Iyer on 02/12/23.
//

#include "IForceField.h"

IForceField::IForceField() {}

Eigen::Vector3d IForceField::getForce(Eigen::Vector3d &loc) const {
    return Eigen::Vector3d(0,0,0);
}