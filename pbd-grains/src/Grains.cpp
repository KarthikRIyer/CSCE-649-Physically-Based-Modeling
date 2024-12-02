//
// Created by Karthik Iyer on 01/12/24.
//

#include "Grains.h"

#include "Particle.h"
#include "Shape.h"

Grains::Grains(int numberOfGrains, double r, double m, std::shared_ptr<Shape> shape) {
    int nx = 10;
    int ny = 20;
    int nz = 10;

    Eigen::Vector3d startPos(-(nx * 2 * r)/2, 0.5, -(nz * 2 * r)/2);

    particles.reserve(nx * ny * nz);

    for (int i = 0; i < nx; i++) {
        for (int j = 0; j < ny; j++) {
            for (int k = 0; k < nz; k++) {
                double x = i * (2 * r);
                double y = j * (2 * r);
                double z = k * (2 * r);
                std::shared_ptr<Particle> p = std::make_shared<Particle>(shape, true);
                p->x0 = startPos +Eigen::Vector3d(x, y, z);
                p->x = p->x0;
                p->v0 = Eigen::Vector3d();
                p->v = p->v0;
                p->r = r;
                p->m = m;
                particles.push_back(p);
            }
        }
    }
}

void Grains::init() {

}

void Grains::reset() {
    for (const std::shared_ptr<Particle>& p: particles) {
        p->reset();
    }
}

void Grains::step(double h, std::vector<std::shared_ptr<IForceField>> &forceFields, SimParams &simParams) {

}

void Grains::draw(std::shared_ptr<MatrixStack> MV, const std::shared_ptr<Program> prog) const {
    for (const std::shared_ptr<Particle>& p: particles) {
        p->draw(MV, prog);
    }
}
