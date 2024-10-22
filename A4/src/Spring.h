//
// Created by Karthik Iyer on 21/10/24.
//
#pragma once
#ifndef A4_SPRING_H
#define A4_SPRING_H

#include <memory>

class Particle;


class Spring {
public:
    Spring(std::shared_ptr<Particle> p0, std::shared_ptr<Particle> p1, double k, double d);
    virtual ~Spring();
    double getRestLength() const;
    double getCurrentLength() const;

    std::shared_ptr<Particle> p0;
    std::shared_ptr<Particle> p1;
    double k; // spring const
    double d; // damper const
    double l; // rest length
};


#endif //A4_SPRING_H
