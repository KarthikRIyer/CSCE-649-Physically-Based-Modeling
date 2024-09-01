//
// Created by Karthik Iyer on 02/12/23.
//

#ifndef HAIRSIM_IFORCEFIELD_H
#define HAIRSIM_IFORCEFIELD_H


#include <Eigen/Core>

class IForceField {
public:
    IForceField();
    virtual Eigen::Vector3d getForce(Eigen::Vector3d &loc) const;
};


#endif //HAIRSIM_IFORCEFIELD_H
