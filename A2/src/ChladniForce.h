//
// Created by Karthik Iyer on 24/09/24.
//

#ifndef A2_CHLADNIFORCE_H
#define A2_CHLADNIFORCE_H


#include "IForceField.h"

class ChladniForce : public IForceField {
public:
    ChladniForce();
    virtual Eigen::Vector3d getForce(Eigen::Vector3d &loc) const;
    void setA(double a);
    void setB(double b);
    void setM(double m);
    void setN(double n);
private:
    double a;
    double b;
    double m;
    double n;
};


#endif //A2_CHLADNIFORCE_H
