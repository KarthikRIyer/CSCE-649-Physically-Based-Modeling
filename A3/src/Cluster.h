//
// Created by Karthik Iyer on 29/09/24.
//
#pragma  once
#ifndef A3_CLUSTER_H
#define A3_CLUSTER_H

#include <vector>
#include <Eigen/Core>

class Particle;
class Shape;
class MatrixStack;
class Program;
class Texture;
class IForceField;
struct SimParams;

class Cluster {
public:
    Cluster(unsigned int count, const Eigen::Vector3d& center,
            double radius, const std::shared_ptr<Shape>& s, std::shared_ptr<Texture> tex);
    void draw(std::shared_ptr<MatrixStack> MV, std::shared_ptr<Program> prog) const;
    void step(double h, std::vector<std::shared_ptr<IForceField>>& forceFields, SimParams& simParams);
private:
    std::shared_ptr<Texture> texture;
    std::vector<std::shared_ptr<Particle>> boids;
};


#endif //A3_CLUSTER_H
