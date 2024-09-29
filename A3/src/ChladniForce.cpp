//
// Created by Karthik Iyer on 24/09/24.
//

#include "ChladniForce.h"
#include <iostream>
#include <Eigen/Dense>


ChladniForce::ChladniForce() {
    a = 0.0;
    b = 0.0;
    m = 0.0;
    n = 0.0;
}
Eigen::Vector3d ChladniForce::getForce(Eigen::Vector3d &loc) const {
    double x = loc.x();
    double z = loc.z();

    double dydx = M_PI * (a * n * std::cos(M_PI * n * x) * std::sin(M_PI * m * z) + b * m * std::sin(M_PI * n * z) * std::cos(M_PI * m * x));
    double dydz = M_PI * (a * m * std::sin(M_PI * n * x) * std::cos(M_PI * m * z) + b * n * std::cos(M_PI * n * z) * std::sin(M_PI * m * x));

    Eigen::Vector3d tx(1.0, dydx, 0.0);
    Eigen::Vector3d tz(0.0, dydz, 1.0);
    Eigen::Vector3d N = tx.cross(tz);
    N = Eigen::Vector3d (N.x(), 0.0, N.z());
    return N;
//    double xGrad = M_PI * ((a * n * std::cos(M_PI * n * x) * std::sin(M_PI * m * z)) + (b * m * std::cos(M_PI * m * x) * std::sin(M_PI * n * z)));
//    double zGrad = M_PI * ((b * n * std::cos(M_PI * n * z) * std::sin(M_PI * m * x)) + (a * m * std::cos(M_PI * m * z) * std::sin(M_PI * n * x)));
//    Eigen::Vector3d grad(xGrad, 0.0, zGrad);
//    std::cout<<"xgrad: "<<xGrad<<" zgrad: "<< zGrad<<"\n";
//    std::cout<<"chladni force: "<<grad.transpose()<<"\n";
//    return -grad.normalized();
}

void ChladniForce::setA(double aVal) {
    this->a = aVal;
}

void ChladniForce::setB(double bVal) {
    this->b = bVal;
}

void ChladniForce::setM(double mVal) {
    this->m = mVal;
}

void ChladniForce::setN(double nVal) {
    this->n = nVal;
}