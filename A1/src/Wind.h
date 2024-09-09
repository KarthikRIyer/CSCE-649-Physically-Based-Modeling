//
// Created by Karthik Iyer on 02/12/23.
//

#ifndef A1_WIND_H
#define A1_WIND_H

#include "IForceField.h"

class Wind : public IForceField {
public:
    Wind(double strength, Eigen::Vector3d dir);
    virtual Eigen::Vector3d getForce(Eigen::Vector3d &loc) const;
    void setStrength(double strength);
    void setDirection(Eigen::Vector3d dir);
private:
    double strength;
    Eigen::Vector3d dir;
};


#endif //A1_WIND_H
