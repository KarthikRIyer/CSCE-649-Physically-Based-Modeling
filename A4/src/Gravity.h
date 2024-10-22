//
// Created by Karthik Iyer on 02/12/23.
//

#ifndef A1_GRAVITY_H
#define A1_GRAVITY_H


#include "IForceField.h"

class Gravity : public IForceField {

public:
    Gravity(Eigen::Vector3d grav);
    virtual Eigen::Vector3d getForce(Eigen::Vector3d &loc) const;
    void setGravity(Eigen::Vector3d grav);
private:
    Eigen::Vector3d grav;
};


#endif //A1_GRAVITY_H
