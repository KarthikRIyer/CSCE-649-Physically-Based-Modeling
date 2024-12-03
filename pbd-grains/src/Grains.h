//
// Created by Karthik Iyer on 01/12/24.
//
#pragma once
#ifndef PBD_GRAINS_GRAINS_H
#define PBD_GRAINS_GRAINS_H

#include <Eigen/Dense>
#include "SpatialHash.h"

class Shape;
class Program;
class MatrixStack;
class IForceField;
class Particle;
struct SimParams;

class Grains {
public:
    Grains(int numberOfGrains, double r, double m, std::shared_ptr<Shape> shape);
    Grains(std::vector<Eigen::Vector3d>& points, double r, double m, std::shared_ptr<Shape> shape);
    void step(double h, std::vector<std::shared_ptr<IForceField>>& forceFields,
              std::vector<std::shared_ptr<Shape> >& shapes, SimParams& simParams);
    void draw(std::shared_ptr<MatrixStack> MV, const std::shared_ptr<Program> p) const;
    void init();
    void reset();
private:
    std::vector<std::shared_ptr<Particle>> particles;
    double xmin;
    double xmax;
    double ymin;
    double ymax;
    double zmin;
    double zmax;

    std::vector<std::pair<int, int>> collisionPairs;
    std::vector<int> collisionCount;
    std::vector<Eigen::Vector3d> dx;

    SpatialHash spatialHash;
};


#endif //PBD_GRAINS_GRAINS_H
