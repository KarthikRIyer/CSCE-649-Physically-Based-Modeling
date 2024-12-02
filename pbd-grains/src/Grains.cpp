//
// Created by Karthik Iyer on 01/12/24.
//

#include "Grains.h"

#include <iostream>

#include "Particle.h"
#include "Shape.h"
#include "IForceField.h"

#define COLL_EPSILON 1e-3
#define SOLVER_ITERATIONS 2

#define MU_K 0.35
#define MU_S 0.3

Grains::Grains(int numberOfGrains, double r, double m, std::shared_ptr<Shape> shape) {
    int nx = 10;
    int ny = 10;
    int nz = 10;

    Eigen::Vector3d startPos(-(nx * 2 * r)/2, 0.5, -(nz * 2 * r)/2);

    particles.reserve(nx * ny * nz);

    xmin = -nx * 2*r;
    xmax = nx * 2*r;
    ymin = 0;
    ymax = 1e10;
    zmin = -nz * 2 * r;
    zmax = nz * 2 * r;

    for (int i = 0; i < nx; i++) {
        for (int j = 0; j < ny; j++) {
            for (int k = 0; k < nz; k++) {
                double x = i * (2 * r) + 0.5 * r * (rand()/double(RAND_MAX));
                double y = j * (2 * r) + 0.5 * r * (rand()/double(RAND_MAX));
                double z = k * (2 * r) + 0.5 * r * (rand()/double(RAND_MAX));
//                double x = i * (2 * r);
//                double y = j * (2 * r);
//                double z = k * (2 * r);
                std::shared_ptr<Particle> p = std::make_shared<Particle>(shape, true);
                p->x0 = startPos +Eigen::Vector3d(x, y, z);
                p->x = p->x0;
                p->v0 = Eigen::Vector3d(0, 0, 0);
                p->v = p->v0;
                p->r = r;
                p->m = m;
                particles.push_back(p);
            }
        }
    }

//    std::shared_ptr<Particle> p = std::make_shared<Particle>(shape, true);
//    p->x0 = Eigen::Vector3d(0.0, 0.5, 0.0);
//    p->x = p->x0;
//    p->v0 = Eigen::Vector3d(0, 0, 0);
//    p->v = p->v0;
//    p->r = r;
//    p->m = m;
//    particles.push_back(p);
//    std::shared_ptr<Particle> p1 = std::make_shared<Particle>(shape, true);
//    p1->x0 = Eigen::Vector3d(2*r - 1e-2, 0.5, 0.0);
//    p1->x = p1->x0;
//    p1->v0 = Eigen::Vector3d(0, 0, 0);
//    p1->v = p1->v0;
//    p1->r = r;
//    p1->m = m;
//    particles.push_back(p1);


    collisionCount.clear();
    collisionPairs.reserve(particles.size() * particles.size());
    collisionCount = std::vector<int>(particles.size(), 0);
    dx = std::vector<Eigen::Vector3d>(particles.size(), Eigen::Vector3d(0, 0, 0));
    reset();
}

void Grains::init() {

}

void Grains::reset() {
    for (const std::shared_ptr<Particle>& p: particles) {
        p->reset();
    }
}

void Grains::step(double h, std::vector<std::shared_ptr<IForceField>> &forceFields, SimParams &simParams) {
    // apply external forces
    for (std::shared_ptr<Particle> p: particles) {
        Eigen::Vector3d f(0, 0, 0);
        for (std::shared_ptr<IForceField> forceField: forceFields) {
            f += forceField->getForce(p->x);
        }

        p->v += (h * f);
        p->xTemp = p->x;
        p->xTemp += (h * p->v);
//        p->x += (h * p->v);
    }
    // boundary collisions
    for (std::shared_ptr<Particle> p: particles) {
        if (p->xTemp.x() < xmin + p->r) {
            p->xTemp += Eigen::Vector3d(xmin + p->r - p->xTemp.x() + COLL_EPSILON, 0, 0);
        }
        if (p->xTemp.x() > xmax - p->r) {
            p->xTemp -= Eigen::Vector3d( p->xTemp.x() - (xmax - p->r) - COLL_EPSILON, 0, 0);
        }
        if (p->xTemp.z() < zmin + p->r) {
            p->xTemp += Eigen::Vector3d(0, 0, zmin + p->r - p->xTemp.z() + COLL_EPSILON);
        }
        if (p->xTemp.z() > zmax - p->r) {
            p->xTemp -= Eigen::Vector3d(0, 0, p->xTemp.z() - (zmax - p->r) - COLL_EPSILON);
        }
        if (p->xTemp.y() < ymin + p->r) {
            p->xTemp += Eigen::Vector3d(0, ymin + p->r - p->xTemp.y() + COLL_EPSILON, 0);
//            p->v.y() *= -1;
        }
    }

    for (int solverIter = 0; solverIter < SOLVER_ITERATIONS; ++solverIter) {
        // find collisions and create constraints
        collisionPairs.clear();
        for (int i = 0; i < collisionCount.size(); ++i) {
            collisionCount[i] = 0;
        }
        for (int i = 0; i < dx.size(); ++i) {
            dx[i] = Eigen::Vector3d(0, 0, 0);
        }
        for (int i = 0; i < particles.size(); ++i) {
            for (int j = i+1; j < particles.size(); ++j) {
                std::shared_ptr<Particle> p0 = particles[i];
                std::shared_ptr<Particle> p1 = particles[j];

                Eigen::Vector3d r = p0->xTemp - p1->xTemp;
                double d = r.norm() - (p0->r + p1->r);
                if (d < 0.0) {
                    collisionPairs.emplace_back(i, j);
                    collisionCount[i]++;
                    collisionCount[j]++;
                }
            }
        }

        // solve constraints
        for (std::pair<int, int> collPair: collisionPairs) {
            std::shared_ptr<Particle> p0 = particles[collPair.first];
            std::shared_ptr<Particle> p1 = particles[collPair.second];

            Eigen::Vector3d r = p0->xTemp - p1->xTemp;
            double d = r.norm() - (p0->r + p1->r);
            double totalInvMass = p0->invM + p1->invM;

            double lambda = d / totalInvMass;
            Eigen::Vector3d normal = r.normalized();
            Eigen::Vector3d dx0 = -p0->invM * lambda * normal;
            Eigen::Vector3d dx1 = p1->invM * lambda * normal;

            dx0 /= collisionCount[collPair.first];
            dx1 /= collisionCount[collPair.second];

            p0->xTemp += dx0;
            p1->xTemp += dx1;

//            dx[collPair.first] += dx0;
//            dx[collPair.second] += dx1;

            // friction
//            Eigen::Vector3d dpf = (p1->xTemp - p1->x) - (p0->xTemp - p0->x);
//            Eigen::Vector3d dpt = dpf - (dpf.dot(normal) * normal);
//            double ldpt = dpt.norm();
//
//            if (ldpt < MU_S * -d) {
//                p0->xTemp -= dpt * (p0->invM / totalInvMass);
//                p1->xTemp += dpt * (p1->invM / totalInvMass);
//            } else {
//                Eigen::Vector3d delta = dpt * std::min(MU_K * -d / ldpt, 1.0);
//                p0->xTemp -= delta * (p0->invM / totalInvMass);
//                p1->xTemp += delta * (p1->invM / totalInvMass);
//            }
        }
//        for (int i = 0; i < particles.size(); ++i) {
//            particles[i]->xTemp += dx[i];
//        }
    }

    // update pos and vel
    for (int i = 0; i < particles.size(); ++i) {
        particles[i]->v = (particles[i]->xTemp - particles[i]->x) / h;
        particles[i]->x = particles[i]->xTemp;
    }
}

void Grains::draw(std::shared_ptr<MatrixStack> MV, const std::shared_ptr<Program> prog) const {
    for (const std::shared_ptr<Particle>& p: particles) {
        p->draw(MV, prog);
    }
//    for (int i = 0; i < 10; ++i) {
//        particles[i]->draw(MV, prog);
//    }
}
