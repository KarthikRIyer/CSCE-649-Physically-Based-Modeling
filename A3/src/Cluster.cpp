//
// Created by Karthik Iyer on 29/09/24.
//

#include <random>
#include <iostream>

#include <tbb/tbb.h>

#include "Cluster.h"
#include "Particle.h"
#include "Shape.h"
#include "Program.h"
#include "MatrixStack.h"
#include "Texture.h"
#include "IForceField.h"
#include "SimParams.h"

Cluster::Cluster(unsigned int count, const Eigen::Vector3d& center,
        double radius, const std::shared_ptr<Shape>& s, std::shared_ptr<Texture> tex): texture(tex) {
    std::mt19937_64 rng;
    uint64_t randomSeed = 123456;
    std::seed_seq ss{uint32_t(randomSeed & 0xffffffff), uint32_t(randomSeed >> 32)};
    rng.seed(ss);
    std::uniform_real_distribution<double> dis(-1.0, 1.0);
    std::uniform_real_distribution<double> dis_radius(0.0, 1.0);

    for (unsigned int i = 0; i < count; i++) {
        std::cout<<"count: "<<i<<"\n";
        double x = dis(rng);
        double y = dis(rng);
        double z = dis(rng);
        double r = std::cbrt(dis_radius(rng)) * radius;
        double magnitude = std::sqrt(x*x + y*y + z*z);
        x /= magnitude;
        y /= magnitude;
        z /= magnitude;
        x *= r;
        y *= r;
        z *= r;
        auto boid = std::make_shared<Particle>(s, true);
        boid->fixed = false;
        boid->r = 0.1;
        boid->x0 = center + Eigen::Vector3d(x, y, z);
        boid->x = boid->x0;
//        boid->v0 = n * simParams.initialParticleVelocity;
        boids.push_back(boid);
    }
}

void Cluster::step(double h, std::vector<std::shared_ptr<IForceField>>& forceFields, SimParams& simParams) {
//    tbb::parallel_for((size_t)0, boids.size(), [&](size_t i) {
//        auto s = boids[i];
//        s->step(h, forceFields, simParams);
//    });

    for (const auto& s : boids) {
        s->step(h, forceFields, simParams);
    }
}

void Cluster::draw(std::shared_ptr<MatrixStack> MV, const std::shared_ptr<Program> prog) const {
    texture->bind(prog->getUniform("kdTex"));
    glUniform3f(prog->getUniform("ka"), 0.1f, 0.1f, 0.1f);
    glUniform3f(prog->getUniform("ks"), 0.1f, 0.1f, 0.1f);
    glUniform1f(prog->getUniform("s"), 200.0f);
    glUniform3fv(prog->getUniform("kdFront"), 1, Eigen::Vector3f(1.0, 1.0, 1.0).data());
    for(int i = 0; i < (int)boids.size(); ++i) {
        boids[i]->draw(MV, prog);
    }
    texture->unbind();
}