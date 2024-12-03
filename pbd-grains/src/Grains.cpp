//
// Created by Karthik Iyer on 01/12/24.
//

#include "Grains.h"

#include <iostream>
#include <tbb/tbb.h>

#include "Particle.h"
#include "Shape.h"
#include "IForceField.h"
#include "SimParams.h"

#define COLL_EPSILON 1e-3
#define LENGTH_EPSILON 1e-6
#define SOLVER_ITERATIONS 2

#define MU_K 0.35
#define MU_S 0.3

Grains::Grains(int numberOfGrains, double r, double m, std::shared_ptr<Shape> shape) {
    int nx = 10;
    int ny = 10;
    int nz = 10;

    int nxs = 20;
    int nzs = 20;

    Eigen::Vector3d startPos(-(nx * 2 * r)/2, 1.5, -(nz * 2 * r)/2);
    Eigen::Vector3d startPos2(-(nxs * 2 * r), 0.0, -(nzs * 2 * r));

    particles.reserve(nx * ny * nz);

    xmin = -nxs * 2*r;
    xmax = nxs * 2*r;
    ymin = 0;
    ymax = 1e10;
    zmin = -nzs * 2 * r;
    zmax = nzs * 2 * r;
    int particleIndex = 0;
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
                p->fixed = false;
                p->i = particleIndex++;
                particles.push_back(p);
            }
        }
    }
    for (int i = 0; i < nxs*2; i++) {
        for (int j = 0; j < nzs*2; j++) {
            double x = i * (2 * r);
            double y = 0;
            double z = j * (2 * r);
            std::shared_ptr<Particle> p = std::make_shared<Particle>(shape, true);
            p->x0 = startPos2 +Eigen::Vector3d(x, y, z);
            p->x = p->x0;
            p->v0 = Eigen::Vector3d(0, 0, 0);
            p->v = p->v0;
            p->r = r;
            p->m = m;
            p->fixed = true;
            p->i = particleIndex++;
            particles.push_back(p);
        }
    }

//    std::shared_ptr<Particle> p = std::make_shared<Particle>(shape, true);
//    p->x0 = Eigen::Vector3d(0.0, r, 0.0);
//    p->x = p->x0;
//    p->v0 = Eigen::Vector3d(0, 0, 0);
//    p->v = p->v0;
//    p->r = r;
//    p->m = m;
//    particles.push_back(p);
//    std::shared_ptr<Particle> p1 = std::make_shared<Particle>(shape, true);
////    p1->x0 = Eigen::Vector3d(2*r - 1e-2, 0.5, 0.0);
//    p1->x0 = p->x0 + 2*r*Eigen::Vector3d(1, 1, 0).normalized();
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

    spatialHash = SpatialHash(2 * r, particles.size());
}

Grains::Grains(std::vector<Eigen::Vector3d>& points, double r, double m, std::shared_ptr<Shape> shape) {

    int nxs = 30;
    int nzs = 30;

//    Eigen::Vector3d startPos(-(nx * 2 * r)/2, 1.5, -(nz * 2 * r)/2);
    Eigen::Vector3d startPos2(-(nxs * 2 * r), 0.0, -(nzs * 2 * r));

    particles.reserve(points.size());
    int particleIndex = 0;
    for (Eigen::Vector3d& pt: points) {
        std::shared_ptr<Particle> p = std::make_shared<Particle>(shape, true);
        p->x0 = pt;
        p->x = p->x0;
        p->v0 = Eigen::Vector3d(0, 0, 0);
        p->v = p->v0;
        p->r = r;
        p->m = m;
        p->fixed = false;
        p->i = particleIndex++;
        particles.push_back(p);
    }

    xmin = -nxs * 2*r;
    xmax = nxs * 2*r;
    ymin = -1;
    ymax = 1e10;
    zmin = -nzs * 2 * r;
    zmax = nzs * 2 * r;
    for (int i = 0; i < nxs*2; i++) {
        for (int j = 0; j < nzs*2; j++) {
            double x = i * (2 * r);
            double y = ymin;
            double z = j * (2 * r);
            std::shared_ptr<Particle> p = std::make_shared<Particle>(shape, true);
            p->x0 = startPos2 +Eigen::Vector3d(x, y, z);
            p->x = p->x0;
            p->v0 = Eigen::Vector3d(0, 0, 0);
            p->v = p->v0;
            p->r = r;
            p->m = m;
            p->fixed = true;
            p->i = particleIndex++;
            particles.push_back(p);
        }
    }

    collisionCount.clear();
    collisionPairs.reserve(particles.size() * particles.size());
    collisionCount = std::vector<int>(particles.size(), 0);
    dx = std::vector<Eigen::Vector3d>(particles.size(), Eigen::Vector3d(0, 0, 0));
    reset();

    spatialHash = SpatialHash(2 * r, particles.size());
}

void Grains::init() {

}

void Grains::reset() {
    for (const std::shared_ptr<Particle>& p: particles) {
        p->reset();
    }
}

void Grains::step(double h, std::vector<std::shared_ptr<IForceField>> &forceFields,
                  std::vector<std::shared_ptr<Shape> >& shapes, SimParams &simParams) {
    // apply external forces
    for (std::shared_ptr<Particle> p: particles) {
        p->xTemp = p->x;
        assert(p->xTemp.x() == p->xTemp.x());
        assert(p->xTemp.y() == p->xTemp.y());
        assert(p->xTemp.z() == p->xTemp.z());
    }
    for (std::shared_ptr<Particle> p: particles) {
        if (p->fixed) continue;
        Eigen::Vector3d f(0, 0, 0);
        for (std::shared_ptr<IForceField> forceField: forceFields) {
            f += forceField->getForce(p->x);
        }

        p->v += (h * f);
//        p->xTemp = p->x;
        p->xTemp += (h * p->v);
//        p->x += (h * p->v);
        assert(p->xTemp.x() == p->xTemp.x());
        assert(p->xTemp.y() == p->xTemp.y());
        assert(p->xTemp.z() == p->xTemp.z());
    }
    // boundary collisions
//    for (std::shared_ptr<Particle> p: particles) {
//        if (p->fixed) continue;
//        if (p->xTemp.x() < xmin + p->r) {
//            p->xTemp += Eigen::Vector3d(xmin + p->r - p->xTemp.x() + COLL_EPSILON, 0, 0);
//        }
//        if (p->xTemp.x() > xmax - p->r) {
//            p->xTemp -= Eigen::Vector3d( p->xTemp.x() - (xmax - p->r) - COLL_EPSILON, 0, 0);
//        }
//        if (p->xTemp.z() < zmin + p->r) {
//            p->xTemp += Eigen::Vector3d(0, 0, zmin + p->r - p->xTemp.z() + COLL_EPSILON);
//        }
//        if (p->xTemp.z() > zmax - p->r) {
//            p->xTemp -= Eigen::Vector3d(0, 0, p->xTemp.z() - (zmax - p->r) - COLL_EPSILON);
//        }
////        if (p->xTemp.y() < ymin + p->r) {
////            p->xTemp += Eigen::Vector3d(0, ymin + p->r - p->xTemp.y() + COLL_EPSILON, 0);
//////            p->v.y() *= -1;
////        }
//        assert(p->xTemp.x() == p->xTemp.x());
//        assert(p->xTemp.y() == p->xTemp.y());
//        assert(p->xTemp.z() == p->xTemp.z());
//    }
    for (std::shared_ptr<Particle> p: particles) {
        p->detectCollision(h, shapes);
        if (p->didCollide) {
            p->didCollide = false;
            p->xTemp = p->xc + (p->nc * p->r);
            p->xTemp += (COLL_EPSILON * p->nc);
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

        // create spatial hash
        spatialHash.create(particles);

        for (int i = 0; i < particles.size(); ++i) {
            spatialHash.query(particles, i, 2 * particles[i]->r);
            for (int ni = 0; ni < spatialHash.getQuerySize(); ++ni) {
                int j = spatialHash.getQueryIds()[ni];
                std::shared_ptr<Particle> p0 = particles[i];
                std::shared_ptr<Particle> p1 = particles[j];
                if (p0->fixed && p1->fixed) continue;
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
//        tbb::parallel_for((size_t)0, collisionPairs.size(), [=](size_t i) {
        for (std::pair<int, int> collPair: collisionPairs) {
//            std::pair<int, int> collPair = collisionPairs[i];
            std::shared_ptr<Particle> p0 = particles[collPair.first];
            std::shared_ptr<Particle> p1 = particles[collPair.second];

            if (!p0->fixed && p1->fixed) {
                std::shared_ptr<Particle> pt = p0;
                p0 = p1;
                p1 = pt;
            }

            if (p0->fixed && !p1->fixed) {
                Eigen::Vector3d r = p0->xTemp - p1->xTemp;
                double d = r.norm() - (p0->r + p1->r);
                double totalInvMass = 1;

                double lambda = d / totalInvMass;
                Eigen::Vector3d normal = r.normalized();
//                Eigen::Vector3d dx0 =  lambda * normal;
                Eigen::Vector3d dx1 = lambda * normal;
                assert(dx1.x() == dx1.x());
                assert(dx1.y() == dx1.y());
                assert(dx1.z() == dx1.z());
//                dx0 /= collisionCount[collPair.first];
                dx1 /= collisionCount[collPair.second];

//                p0->xTemp += dx0;
                p1->xTemp += dx1;

//            dx[collPair.first] += dx0;
//            dx[collPair.second] += dx1;

                // friction
                Eigen::Vector3d dpf = (p1->xTemp - p1->x) - (p0->xTemp - p0->x);
                Eigen::Vector3d dpt = dpf - (dpf.dot(normal) * normal);
                double ldpt = dpt.norm();
                if (ldpt < LENGTH_EPSILON) continue;

                if (ldpt < simParams.staticFrictionCoeff * -d) {
//                    p0->xTemp += (dpt * (p0->invM / totalInvMass));
                    p1->xTemp -= (dpt * 1);
                } else {
                    Eigen::Vector3d delta = dpt * std::min(simParams.kineticFrictionCoeff * -d / ldpt, 1.0);
//                    p0->xTemp += (delta * (p0->invM / totalInvMass));
                    p1->xTemp -= (delta * 1);
//                p0->xTemp += (dpt * (p0->invM / totalInvMass));
//                p1->xTemp -= (dpt * (p1->invM / totalInvMass));
                }
            } else {
                Eigen::Vector3d r = p0->xTemp - p1->xTemp;
                double d = r.norm() - (p0->r + p1->r);
                double totalInvMass = p0->invM + p1->invM;

                double lambda = d / totalInvMass;
                Eigen::Vector3d normal = r.normalized();
                Eigen::Vector3d dx0 = -p0->invM * lambda * normal;
                Eigen::Vector3d dx1 = p1->invM * lambda * normal;

                assert(collisionCount[collPair.first] != 0);
                assert(collisionCount[collPair.second] != 0);

                dx0 /= collisionCount[collPair.first];
                dx1 /= collisionCount[collPair.second];

                assert(dx0.x() == dx0.x());
                assert(dx0.y() == dx0.y());
                assert(dx0.z() == dx0.z());
                assert(dx1.x() == dx1.x());
                assert(dx1.y() == dx1.y());
                assert(dx1.z() == dx1.z());

                p0->xTemp += dx0;
                p1->xTemp += dx1;

                assert(p0->xTemp.x() == p0->xTemp.x());
                assert(p0->xTemp.y() == p0->xTemp.y());
                assert(p0->xTemp.z() == p0->xTemp.z());
                assert(p1->xTemp.x() == p1->xTemp.x());
                assert(p1->xTemp.y() == p1->xTemp.y());
                assert(p1->xTemp.z() == p1->xTemp.z());

//            dx[collPair.first] += dx0;
//            dx[collPair.second] += dx1;

                // friction
                Eigen::Vector3d dpf = (p1->xTemp - p1->x) - (p0->xTemp - p0->x);
                Eigen::Vector3d dpt = dpf - (dpf.dot(normal) * normal);
                double ldpt = dpt.norm();
                if (ldpt < LENGTH_EPSILON) continue;

                if (ldpt < MU_S * -d) {
                    p0->xTemp += (dpt * (p0->invM / totalInvMass));
                    p1->xTemp -= (dpt * (p1->invM / totalInvMass));
                } else {
                    Eigen::Vector3d delta = dpt * std::min(MU_K * -d / ldpt, 1.0);
                    p0->xTemp += (delta * (p0->invM / totalInvMass));
                    p1->xTemp -= (delta * (p1->invM / totalInvMass));
//                p0->xTemp += (dpt * (p0->invM / totalInvMass));
//                p1->xTemp -= (dpt * (p1->invM / totalInvMass));
                }
                assert(p0->xTemp.x() == p0->xTemp.x());
                assert(p0->xTemp.y() == p0->xTemp.y());
                assert(p0->xTemp.z() == p0->xTemp.z());
                assert(p1->xTemp.x() == p1->xTemp.x());
                assert(p1->xTemp.y() == p1->xTemp.y());
                assert(p1->xTemp.z() == p1->xTemp.z());
            }
        }
//        });
//        for (int i = 0; i < particles.size(); ++i) {
//            particles[i]->xTemp += dx[i];
//        }
    }

    // update pos and vel
    for (int i = 0; i < particles.size(); ++i) {
        if (particles[i]->fixed) continue;
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
