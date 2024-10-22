//
// Created by Karthik Iyer on 21/10/24.
//

#include "Spring.h"
#include "Particle.h"

Spring::Spring(std::shared_ptr<Particle> p0, std::shared_ptr<Particle> p1, double k, double d): k(k), d(d) {
    assert(p0);
    assert(p1);
    this->p0 = p0;
    this->p1 = p1;

    Eigen::Vector3d  dx = p0->x - p1->x;
    l = dx.norm();
    assert(l > 0.0);
}

Spring::~Spring() = default;

double Spring::getRestLength() const {
    return l;
}

double Spring::getCurrentLength() const {
    Eigen::Vector3d  dx = p0->x - p1->x;
    return dx.norm();
}