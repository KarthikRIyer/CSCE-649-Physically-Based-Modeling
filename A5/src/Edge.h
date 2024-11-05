//
// Created by Karthik Iyer on 21/10/24.
//
#pragma once
#ifndef A4_Edge_H
#define A4_Edge_H

#include <memory>

class Particle;


class Edge {
public:
    Edge(std::shared_ptr<Particle> p0, std::shared_ptr<Particle> p1);
    virtual ~Edge();

    std::shared_ptr<Particle> p0;
    std::shared_ptr<Particle> p1;
};


#endif //A4_Edge_H
