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
    virtual ~IForceField() = 0;
};


#endif //A1_IFORCEFIELD_H
