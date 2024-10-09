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
        double radius, const std::shared_ptr<Shape>& s,
        std::shared_ptr<Texture> tex, SimParams& simParams): texture(tex), radius(radius) {
    std::mt19937_64 rng;
    std::mt19937_64 rng2;
    uint64_t randomSeed = 123456;
    uint64_t randomSeed2 = 12356;
    std::seed_seq ss{uint32_t(randomSeed & 0xffffffff), uint32_t(randomSeed >> 32)};
    std::seed_seq ss2{uint32_t(randomSeed2 & 0xffffffff), uint32_t(randomSeed2 >> 32)};
    rng.seed(ss);
    rng2.seed(ss2);
    std::uniform_real_distribution<double> dis(-1.0, 1.0);
    std::uniform_real_distribution<double> dis_radius(0.0, 1.0);

    leadBoid = std::make_shared<Particle>(s, true);
    leadBoid->r = 0.5;
    leadBoid->x0 = center;
    leadBoid->x = leadBoid->x0;

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

        double vx = dis(rng2);
        double vy = dis(rng2);
        double vz = dis(rng2);
        magnitude = std::sqrt(vx*vx + vy*vy + vz*vz);
        vx /= magnitude;
        vy /= magnitude;
        vz /= magnitude;
        r = std::cbrt(dis_radius(rng2));
        vx *= r;
        vy *= r;
        vz *= r;

        auto boid = std::make_shared<Particle>(s, true);
        boid->fixed = false;
        boid->r = 0.1;
        boid->x0 = center + Eigen::Vector3d(x, y, z);
        boid->x = boid->x0;
//        Eigen::Vector3d randVel(dis(rng2), dis(rng2), dis(rng2));
        Eigen::Vector3d randVel(vx, vy, vz);
//        randVel.normalize();
        boid->v0 = randVel * simParams.initialParticleVelocity;
        boid->v = boid->v0;
        boids.push_back(boid);
    }
    xTemp.resize(count);
    vTemp.resize(count);
    aTemp.resize(count);
}

double Cluster::getWd(double dij) {
    if (dij <= radius * 0.25) return 1.0;
//    return 0.1;
    else if (dij <= radius * 0.5) {
        double slope = -4.0 / radius;
        return 1.0 + slope * (dij - 0.25 * radius);
    }
    return 0.0;
}

Eigen::Vector3d getBoxForce(Eigen::Vector3d pos) {
    double forceMag = 500.0;
    Eigen::Vector3d force(0.0, 0.0, 0.0);
    if (pos.x() >= 9.5 && pos.x() <= 10.0) {
        force.x() -= forceMag;
    } else if (pos.x() <= -9.5 && pos.x() >= -10.0) {
        force.x() += forceMag;
    }

    if (pos.y() >= 9.5 && pos.y() <= 10.0) {
        force.y() -= forceMag;
    } else if (pos.y() <= -9.5 && pos.y() >= -10.0) {
        force.y() += forceMag;
    }

    if (pos.z() >= 9.5 && pos.z() <= 10.0) {
        force.z() -= forceMag;
    } else if (pos.z() <= -9.5 && pos.z() >= -10.0) {
        force.z() += forceMag;
    }

    return force;
}

void Cluster::step(double t, double h, std::vector<std::shared_ptr<IForceField>>& forceFields, SimParams& simParams) {
//    tbb::parallel_for((size_t)0, boids.size(), [&](size_t i) {
//        auto s = boids[i];
//        s->step(h, forceFields, simParams);
//    });

    double lbRad = 5.0;
    double angVel = 1.0;
    double lbX = lbRad * std::cos(angVel * t);
    double lbZ = lbRad * std::sin(angVel * t);
    leadBoid->v = Eigen::Vector3d(-lbRad * angVel * std::sin(angVel * t), 0.0, lbRad * angVel * std::cos(angVel * t));
//    leadBoid->x = Eigen::Vector3d(lbX, leadBoid->x.y(), lbZ);
    leadBoid->x += leadBoid->v * h;

    for (int i = 0; i < boids.size(); i++) {
        auto s = boids[i];
        xTemp[i] = s->x;
        vTemp[i] = s->v;
    }

//    for (int i = 0; i < boids.size(); i++) {
//        Eigen::Vector3d pcj;
//        Eigen::Vector3d pvj;
//        Eigen::Vector3d c;
//        int neighbors = 0;
//        for (int j = 0; j < boids.size(); j++) {
//            if (i == j) continue;
//            Eigen::Vector3d xi = boids[i]->x;
//            Eigen::Vector3d xj = boids[j]->x;
//            Eigen::Vector3d xij = xj - xi;
//            double dij = xij.norm();
////            double wa = (vi.dot(uij) / (vi.norm() * uij.norm()));
//            double wd = getWd(dij);
//            if (wd != 0.0) neighbors++;
//
//            // separation
//            if (dij <= 0.5)
//                pcj -= (boids[j]->x - boids[i]->x);
//
//            //alignment
//            pvj += boids[j]->v;
//
//            c += (boids[j]->x);
//        }
//
//        // calc for lead boid
//        pvj += leadBoid->v;
//        c += leadBoid->x;
//        if ((leadBoid->x - boids[i]->x).norm() <= 0.5) {
//            pcj -= (leadBoid->x - boids[i]->x);
//        }
//
////        pcj /= neighbors;
////        pcj -= boids[i]->x;
//        pvj /= (boids.size());
//        pvj = (pvj - boids[i]->v);
//
//        c /= (boids.size());
//        c -= boids[i]->x;
//        c /= 100;
//
//        Eigen::Vector3d collForce = getBoxForce(boids[i]->x);
////        std::cout<<"collForce: "<<collForce.transpose()<<"\n";
//
//        vTemp[i] += (pcj + pvj + c + collForce) * h;
//        xTemp[i] += vTemp[i] * h;
//    }

    for (int i = 0; i < boids.size(); i++) {
        Eigen::Vector3d aCollAvoid;
        Eigen::Vector3d aVelMatch;
        Eigen::Vector3d aFlockCen;
        for (int j = 0; j < boids.size(); j++) {
            if (i == j) continue;
            Eigen::Vector3d xi = boids[i]->x;
            Eigen::Vector3d xj = boids[j]->x;
            Eigen::Vector3d vi = boids[i]->v;
            Eigen::Vector3d vj = boids[j]->v;
            Eigen::Vector3d xij = xj - xi;
            double dij = xij.norm();
            Eigen::Vector3d uij = xij.normalized();

            double wa = (vi.dot(uij) / (vi.norm() * uij.norm()));
            double wd = getWd(dij);

            aCollAvoid += wd * wa * (-kCOllAvoid * (1.0 / dij) * uij);
//            aCollAvoid += 0.01 * (-kCOllAvoid * (1.0 / dij) * uij);
            aVelMatch += wd * wa * (kVelMatch * (vj - vi));
//            aVelMatch += 0.01 * (kVelMatch * (vj - vi));
            aFlockCen += wd * wa * (kFlockCen * uij);
//            aFlockCen += 0.01 * (kFlockCen * uij);
        }

        Eigen::Vector3d xi = boids[i]->x;
        Eigen::Vector3d xj = leadBoid->x;
        Eigen::Vector3d vi = boids[i]->v;
        Eigen::Vector3d vj = leadBoid->v;
        Eigen::Vector3d xij = xj - xi;
        double dij = xij.norm();
        Eigen::Vector3d uij = xij.normalized();

        double wa = (vi.dot(uij) / (vi.norm() * uij.norm()));
        double wd = getWd(dij);
        aCollAvoid += wd * wa * (-kCOllAvoid * (1.0 / dij) * uij);
        aVelMatch += wd * wa * (kVelMatch * (vj - vi));
        aFlockCen += wd * wa * (kFlockCen * uij);


        Eigen::Vector3d collForce = getBoxForce(boids[i]->x);
        aTemp[i] = (aCollAvoid + aVelMatch + aFlockCen)/(boids.size());
        aTemp[i] += collForce;
    }

    for (int i = 0; i < boids.size(); i++) {
        xTemp[i] += (h * vTemp[i]);
        vTemp[i] += (h * aTemp[i]);
    }

    for (int i = 0; i < boids.size(); i++) {
        boids[i]->x = xTemp[i];
        boids[i]->v = vTemp[i];
    }

//    Eigen::Vector3d vNew = v;
//    Eigen::Vector3d fNet(0, 0, 0);
//    for (auto forceField: forceFields) {
//        fNet += forceField->getForce(x);
//        vNew += (forceField->getForce(x) * h);
//    }
//    vNew -= (h * (simParams.airFrictionFactor / m) * v);
//
//    Eigen::Vector3d xNew = x + (v * h);
//
//    x = xNew;
//    v = vNew;

//    for (const auto& s : boids) {
//        s->step(h, forceFields, simParams);
//    }
}

void Cluster::reset() {
    for (int i = 0; i < boids.size(); i++) {
        boids[i]->x = boids[i]->x0;
        boids[i]->v = boids[i]->v0;
    }
    for (int i = 0; i < aTemp.size(); i++) {
        aTemp[i] = Eigen::Vector3d();
        vTemp[i] = Eigen::Vector3d();
        xTemp[i] = Eigen::Vector3d();
    }
    leadBoid->x = leadBoid->x0;
    leadBoid->v = leadBoid->v0;
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
    leadBoid->draw(MV, prog);
    texture->unbind();
}