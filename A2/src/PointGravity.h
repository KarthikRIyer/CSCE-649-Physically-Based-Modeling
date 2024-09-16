//
// Created by Karthik Iyer on 15/09/24.
//
#pragma once
#ifndef A2_POINTGRAVITY_H
#define A2_POINTGRAVITY_H

#include "IForceField.h"

class PointGravity : public IForceField{
public:
    PointGravity(double mass, Eigen::Vector3d pos);
    virtual Eigen::Vector3d getForce(Eigen::Vector3d &loc) const;
private:
    double mass;
    double G = 6.67e-11;
    Eigen::Vector3d pos;
};


#endif //A2_POINTGRAVITY_H
