//
// Created by Karthik Iyer on 21/10/24.
//

#include "Edge.h"
#include "Particle.h"

Edge::Edge(std::shared_ptr<Particle> p0, std::shared_ptr<Particle> p1) {
    assert(p0);
    assert(p1);
    this->p0 = p0;
    this->p1 = p1;
}

Edge::~Edge() = default;