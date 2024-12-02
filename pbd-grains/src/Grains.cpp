//
// Created by Karthik Iyer on 01/12/24.
//

#include "Grains.h"

#include <iostream>

#include "Particle.h"
#include "Shape.h"
#include "IForceField.h"

#define COLL_EPSILON 1e-3

Grains::Grains(int numberOfGrains, double r, double m, std::shared_ptr<Shape> shape) {
    int nx = 10;
    int ny = 20;
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
                double x = i * (2 * r);
                double y = j * (2 * r);
                double z = k * (2 * r);
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
        p->x += (h * p->v);
    }
    // boundary collisions
    for (std::shared_ptr<Particle> p: particles) {
        if (p->x.x() < xmin + p->r) {
            p->x += Eigen::Vector3d(xmin + p->r - p->x.x() + COLL_EPSILON, 0, 0);
        }
        if (p->x.x() > xmax - p->r) {
            p->x -= Eigen::Vector3d( p->x.x() - (xmax - p->r) - COLL_EPSILON, 0, 0);
        }
        if (p->x.z() < zmin + p->r) {
            p->x += Eigen::Vector3d(0, 0, zmin + p->r - p->x.z() + COLL_EPSILON);
        }
        if (p->x.z() > zmax - p->r) {
            p->x -= Eigen::Vector3d(0, 0, p->x.z() - (zmax - p->r) - COLL_EPSILON);
        }
        if (p->x.y() < ymin + p->r) {
            p->x += Eigen::Vector3d(0, ymin + p->r - p->x.y() + COLL_EPSILON, 0);
//            p->v.y() *= -1;
        }
    }

    // find collisions and create constraints

    // solve constraints

    // update pos
}

void Grains::draw(std::shared_ptr<MatrixStack> MV, const std::shared_ptr<Program> prog) const {
    for (const std::shared_ptr<Particle>& p: particles) {
        p->draw(MV, prog);
    }
}
