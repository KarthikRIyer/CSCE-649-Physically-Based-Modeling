//
// Created by Karthik Iyer on 02/12/23.
//

#ifndef A1_IFORCEFIELD_H
#define A1_IFORCEFIELD_H


#include <Eigen/Core>

class IForceField {
public:
    IForceField();
    virtual Eigen::Vector3d getForce(Eigen::Vector3d &loc) const;
};


#endif //A1_IFORCEFIELD_H
