//
// Created by Karthik Iyer on 21/10/24.
//

#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>

#include <iostream>

#include "SingleSpring.h"
#include "Particle.h"
#include "Spring.h"
#include "GLSL.h"
#include "Program.h"
#include "IForceField.h"
#include "Shape.h"
#include "Polygon.h"
#include "Edge.h"
#include "SimParams.h"

SingleSpring::SingleSpring() {
    double springConst = 500.0;
    double damperConst = 1.0;
    std::shared_ptr<Particle> p0 = std::make_shared<Particle>();
    std::shared_ptr<Particle> p1 = std::make_shared<Particle>();

    p0->x0 = Eigen::Vector3d(-0.4, 1.0, 0.0);
    p1->x0 = Eigen::Vector3d(0.6, 1.0, 0.0);

    particles.push_back(p0);
    particles.push_back(p1);

    for (int i = 0; i < particles.size(); ++i) {
        particles[i]->v0 = Eigen::Vector3d(0.0, 0.0, 0.0);
        particles[i]->v = particles[i]->v0;
        particles[i]->x = particles[i]->x0;
        particles[i]->f = Eigen::Vector3d(0.0, 0.0, 0.0);
        particles[i]->fixed = false;
    }

    std::shared_ptr<Spring> s0 = std::make_shared<Spring>(p0, p1, springConst, damperConst);
    springs.push_back(s0);

    std::shared_ptr<Edge> e0 = std::make_shared<Edge>(p0, p1);
    edges.push_back(e0);

//    for (int i = 0; i < springs.size(); ++i) {
//        springs[i]->d = 0;
//    }
    posBuf.clear();
    eleBuf.clear();
    posBuf.resize(springs.size() * 2 * 3);
    eleBuf.resize(springs.size() * 2);

    int posBufIndex = 0;
    int eleBufIndex = 0;
    int posIndex = 0;
    for (int i = 0; i < springs.size(); ++i) {
        std::shared_ptr<Spring> s = springs[i];
        posBuf[posBufIndex++] = s->p0->x.x();
        posBuf[posBufIndex++] = s->p0->x.y();
        posBuf[posBufIndex++] = s->p0->x.z();

        posBuf[posBufIndex++] = s->p1->x.x();
        posBuf[posBufIndex++] = s->p1->x.y();
        posBuf[posBufIndex++] = s->p1->x.z();

        eleBuf[eleBufIndex++] = posIndex++;
        eleBuf[eleBufIndex++] = posIndex++;
    }

//    p0->x = Eigen::Vector3d(0.5, 0.8, 0.0);
//    p1->x = Eigen::Vector3d(0.5, 1.0, 0.0);
}

SingleSpring::~SingleSpring() = default;

void SingleSpring::tare() {
    for (const auto & particle : particles) {
        particle->tare();
    }
}

void SingleSpring::reset() {
    for (const auto & particle : particles) {
        particle->reset();
    }
    int posBufIndex = 0;
    for (int i = 0; i < springs.size(); ++i) {
        std::shared_ptr<Spring> s = springs[i];
        std::shared_ptr<Particle> p0 = s->p0;
        std::shared_ptr<Particle> p1 = s->p1;
        posBuf[posBufIndex++] = p0->x.x();
        posBuf[posBufIndex++] = p0->x.y();
        posBuf[posBufIndex++] = p0->x.z();

        posBuf[posBufIndex++] = p1->x.x();
        posBuf[posBufIndex++] = p1->x.y();
        posBuf[posBufIndex++] = p1->x.z();
    }
}

std::pair<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d >> SingleSpring::getVelAcc(double h, std::vector<std::shared_ptr<IForceField>>& forceFields) {
    int integrationScheme = 2;
    std::vector<Eigen::Vector3d> v (particles.size(), Eigen::Vector3d());
    std::vector<Eigen::Vector3d> a (particles.size(), Eigen::Vector3d());
    for (const auto & particle : particles) {

        particle->f = Eigen::Vector3d(0.0, 0.0, 0.0);
        particle->fExt = Eigen::Vector3d(0.0, 0.0, 0.0);
        particle->vk1 = Eigen::Vector3d(0.0, 0.0, 0.0);
        particle->vk2 = Eigen::Vector3d(0.0, 0.0, 0.0);
        particle->vk3 = Eigen::Vector3d(0.0, 0.0, 0.0);
        particle->vk4 = Eigen::Vector3d(0.0, 0.0, 0.0);
        particle->ak1 = Eigen::Vector3d(0.0, 0.0, 0.0);
        particle->ak2 = Eigen::Vector3d(0.0, 0.0, 0.0);
        particle->ak3 = Eigen::Vector3d(0.0, 0.0, 0.0);
        particle->ak4 = Eigen::Vector3d(0.0, 0.0, 0.0);

        for (int i = 0; i < forceFields.size(); ++i) {
            std::shared_ptr<IForceField> forceField = forceFields[i];
//            particle->f += forceField->getForce(particle->x);
            particle->fExt += forceField->getForce(particle->x);
            particle->f += particle->fExt;
        }
    }

    for (int i = 0; i < springs.size(); ++i) {
        std::shared_ptr<Spring> spring = springs[i];

        Eigen::Vector3d v0 = spring->p0->v;
        Eigen::Vector3d v1 = spring->p1->v;
        Eigen::Vector3d u01 = (spring->p1->x - spring->p0->x).normalized();
        double currentLength = spring->getCurrentLength();
        double restLength = spring->getRestLength();

        Eigen::Vector3d springF = spring->k * (currentLength - restLength) * u01;
        Eigen::Vector3d damperF = spring->d * ((v1 - v0).dot(u01)) * u01;

        spring->p0->f += springF;
        spring->p0->f += damperF;
        spring->p1->f += -springF;
        spring->p1->f += -damperF;
    }

    if (integrationScheme == 0) {
        for (int i = 0; i < particles.size(); ++i) {
            v[i] = particles[i]->v;
            a[i] = particles[i]->f / particles[i]->m;
        }
    } else if (integrationScheme == 1) {
        for (int i = 0; i < particles.size(); ++i) {
            particles[i]->vk1 = particles[i]->v;
            particles[i]->ak1 = particles[i]->f / particles[i]->m;
        }

        for (int i = 0; i < particles.size(); ++i) {
            particles[i]->xTemp = particles[i]->x + particles[i]->vk1 * h * 0.5;
            particles[i]->vTemp = particles[i]->v + particles[i]->ak1 * h * 0.5;
        }
        for (const auto & particle : particles) {
            particle->f = particle->fExt;
            for (int i = 0; i < springs.size(); ++i) {
                std::shared_ptr<Spring> spring = springs[i];

                Eigen::Vector3d v0 = spring->p0->vTemp;
                Eigen::Vector3d v1 = spring->p1->vTemp;
                Eigen::Vector3d u01 = (spring->p1->xTemp - spring->p0->xTemp).normalized();
                double currentLength = spring->getCurrentLength();
                double restLength = spring->getRestLength();

                Eigen::Vector3d springF = spring->k * (currentLength - restLength) * u01;
                Eigen::Vector3d damperF = spring->d * ((v1 - v0).dot(u01)) * u01;

                spring->p0->f += springF;
                spring->p0->f += damperF;
                spring->p1->f += -springF;
                spring->p1->f += -damperF;
            }
            particle->vk2 = particle->vTemp;
            particle->ak2 = particle->f / particle->m;
        }
        for (int i = 0; i < particles.size(); ++i) {
            v[i] = particles[i]->vk2;
            a[i] = particles[i]->ak2;
        }
    } else if (integrationScheme == 2) {
        // K1
        for (int i = 0; i < particles.size(); ++i) {
            particles[i]->vk1 = particles[i]->v;
            particles[i]->ak1 = particles[i]->f / particles[i]->m;
        }

        // K2
        for (int i = 0; i < particles.size(); ++i) {
            particles[i]->xTemp = particles[i]->x + particles[i]->vk1 * h * 0.5;
            particles[i]->vTemp = particles[i]->v + particles[i]->ak1 * h * 0.5;
        }
        for (const auto & particle : particles) {
            particle->f = particle->fExt;
        }
        for (int i = 0; i < springs.size(); ++i) {
            std::shared_ptr<Spring> spring = springs[i];

            Eigen::Vector3d v0 = spring->p0->vTemp;
            Eigen::Vector3d v1 = spring->p1->vTemp;
            Eigen::Vector3d u01 = (spring->p1->xTemp - spring->p0->xTemp).normalized();
            double currentLength = spring->getCurrentLength();
            double restLength = spring->getRestLength();

            Eigen::Vector3d springF = spring->k * (currentLength - restLength) * u01;
            Eigen::Vector3d damperF = spring->d * ((v1 - v0).dot(u01)) * u01;

            spring->p0->f += springF;
            spring->p0->f += damperF;
            spring->p1->f += -springF;
            spring->p1->f += -damperF;
        }
        for (const auto & particle : particles) {
            particle->vk2 = particle->vTemp;
            particle->ak2 = particle->f / particle->m;
        }

        // K3
        for (int i = 0; i < particles.size(); ++i) {
            particles[i]->xTemp = particles[i]->x + particles[i]->vk2 * h * 0.5;
            particles[i]->vTemp = particles[i]->v + particles[i]->ak2 * h * 0.5;
        }
        for (const auto & particle : particles) {
            particle->f = particle->fExt;
        }
        for (int i = 0; i < springs.size(); ++i) {
            std::shared_ptr<Spring> spring = springs[i];

            Eigen::Vector3d v0 = spring->p0->vTemp;
            Eigen::Vector3d v1 = spring->p1->vTemp;
            Eigen::Vector3d u01 = (spring->p1->xTemp - spring->p0->xTemp).normalized();
            double currentLength = spring->getCurrentLength();
            double restLength = spring->getRestLength();

            Eigen::Vector3d springF = spring->k * (currentLength - restLength) * u01;
            Eigen::Vector3d damperF = spring->d * ((v1 - v0).dot(u01)) * u01;

            spring->p0->f += springF;
            spring->p0->f += damperF;
            spring->p1->f += -springF;
            spring->p1->f += -damperF;
        }
        for (const auto & particle : particles) {
            particle->vk3 = particle->vTemp;
            particle->ak3 = particle->f / particle->m;
        }

        // K4
        for (int i = 0; i < particles.size(); ++i) {
            particles[i]->xTemp = particles[i]->x + particles[i]->vk3 * h;
            particles[i]->vTemp = particles[i]->v + particles[i]->ak3 * h;
        }
        for (const auto & particle : particles) {
            particle->f = particle->fExt;
        }
        for (int i = 0; i < springs.size(); ++i) {
            std::shared_ptr<Spring> spring = springs[i];

            Eigen::Vector3d v0 = spring->p0->vTemp;
            Eigen::Vector3d v1 = spring->p1->vTemp;
            Eigen::Vector3d u01 = (spring->p1->xTemp - spring->p0->xTemp).normalized();
            double currentLength = spring->getCurrentLength();
            double restLength = spring->getRestLength();

            Eigen::Vector3d springF = spring->k * (currentLength - restLength) * u01;
            Eigen::Vector3d damperF = spring->d * ((v1 - v0).dot(u01)) * u01;

            spring->p0->f += springF;
            spring->p0->f += damperF;
            spring->p1->f += -springF;
            spring->p1->f += -damperF;
        }
        for (const auto & particle : particles) {
            particle->vk4 = particle->vTemp;
            particle->ak4 = particle->f / particle->m;
        }

        // Final derivative
        for (int i = 0; i < particles.size(); ++i) {
            v[i] = (1.0 / 6.0) * (particles[i]->vk1 + 2 * particles[i]->vk2 + 2 * particles[i]->vk3 + particles[i]->vk4);
            a[i] = (1.0 / 6.0) * (particles[i]->ak1 + 2 * particles[i]->ak2 + 2 * particles[i]->ak3 + particles[i]->ak4);
        }
        // use the above computed derivates like this
        // auto va = getVellAcc(); // returns pair of velocities and accelerations
        // for (int i = 0; i < particles.size(); ++i) {
        //     particles[i]->x += va.first[i] * h;
        //     particles[i]->v += va.second[i] * h;
        // }
    }

    return std::make_pair(v, a);
}

void SingleSpring::step(double h, std::vector<std::shared_ptr<IForceField>>& forceFields, SimParams& simParams, std::vector<std::shared_ptr<Shape> >& shapes) {
//    for (const auto & particle : particles) {
//        particle->f = Eigen::Vector3d(0.0, 0.0, 0.0);
//        for (int i = 0; i < forceFields.size(); ++i) {
//            std::shared_ptr<IForceField> forceField = forceFields[i];
//            particle->f += forceField->getForce(particle->x);
//        }
//    }
//
//    for (int i = 0; i < springs.size(); ++i) {
//        std::shared_ptr<Spring> spring = springs[i];
//
//        Eigen::Vector3d v0 = spring->p0->v;
//        Eigen::Vector3d v1 = spring->p1->v;
//        Eigen::Vector3d u01 = (spring->p1->x - spring->p0->x).normalized();
//        double currentLength = spring->getCurrentLength();
//        double restLength = spring->getRestLength();
//
//        Eigen::Vector3d springF = spring->k * (currentLength - restLength) * u01;
//        Eigen::Vector3d damperF = spring->d * ((v1 - v0).dot(u01)) * u01;
//
//        spring->p0->f += springF;
//        spring->p0->f += damperF;
//        spring->p1->f += -springF;
//        spring->p1->f += -damperF;
//    }

    auto va = getVelAcc(h, forceFields);

    for (int i = 0; i < particles.size(); ++i) {
        particles[i]->xTemp = particles[i]->x;
        particles[i]->vTemp = particles[i]->v;
        particles[i]->x += va.first[i] * h;
        particles[i]->v += va.second[i] * h;
        particles[i]->didCollide = false;
    }

    for (int i = 0; i < particles.size(); ++i) {
        detectCollision(particles[i], shapes, simParams);
    }

    for (int i = 0; i < edges.size(); ++i) {
        if (edges[i]->p0->didCollide && edges[i]->p1->didCollide)
            continue;
//        edges[i]->p0->xTemp = edges[i]->p0->x;
//        edges[i]->p1->xTemp = edges[i]->p1->x;
//        edges[i]->p0->vTemp = edges[i]->p0->v;
//        edges[i]->p1->vTemp = edges[i]->p1->v;
        detectEdgeCollision(edges[i], shapes, simParams);
    }

    int posBufIndex = 0;
    for (int i = 0; i < springs.size(); ++i) {
        std::shared_ptr<Spring> s = springs[i];
        std::shared_ptr<Particle> p0 = s->p0;
        std::shared_ptr<Particle> p1 = s->p1;
        posBuf[posBufIndex++] = p0->x.x();
        posBuf[posBufIndex++] = p0->x.y();
        posBuf[posBufIndex++] = p0->x.z();

        posBuf[posBufIndex++] = p1->x.x();
        posBuf[posBufIndex++] = p1->x.y();
        posBuf[posBufIndex++] = p1->x.z();
    }
}

double SingleSpring::sgn(double x) {
    if (x > 0) return 1;
    if (x < 0) return -1;
    return 0;
}

void SingleSpring::detectEdgeCollision(std::shared_ptr<Edge> edge, std::vector<std::shared_ptr<Shape>> &shapes,
                                       SimParams &simParams) {
    std::shared_ptr<Particle> p0 = edge->p0;
    std::shared_ptr<Particle> p1 = edge->p1;

    Eigen::Vector3d e1 = p1->xTemp - p0->xTemp;
    Eigen::Vector3d P1 = p0->xTemp;

    Eigen::Vector3d e1Plus = p1->x - p0->x;
    Eigen::Vector3d P1Plus = p0->x;

    for (auto shape: shapes) {
        if (!shape->getObstacle()) {
            continue;
        }

        for (Polygon p: shape->getPolygons()) {
            Eigen::Vector3d P = p.points[0];
            Eigen::Vector3d Q = p.points[1];
            Eigen::Vector3d R = p.points[2];

            // PQ
            Eigen::Vector3d e2 = Q - P;
            Eigen::Vector3d n = e1.cross(e2);
            Eigen::Vector3d q = P - P1;
            n.normalize();
            double s = (q.dot(e2.normalized().cross(n))) / (e1.dot(e2.normalized().cross(n)));
            double t = (-q.dot(e1.normalized().cross(n))) / (e2.dot(e1.normalized().cross(n)));

            Eigen::Vector3d nPlus = e1Plus.cross(e2);
            nPlus.normalize();
            Eigen::Vector3d qPlus = P - P1Plus;
            double sPlus = (qPlus.dot(e2.normalized().cross(nPlus))) / (e1Plus.dot(e2.normalized().cross(nPlus)));
            double tPlus = (-qPlus.dot(e1Plus.normalized().cross(nPlus))) / (e2.dot(e1Plus.normalized().cross(nPlus)));

            if (s >= 0 && s <= 1 && t >= 0 && t <= 1) {
                Eigen::Vector3d collPos = P1 + s * e1;
                Eigen::Vector3d collPos2 = P + t * e2;
                double dist = (collPos2 - collPos).norm();
//                    std::cout<<"Before: " << collPos.transpose() << "\n";
//                    std::cout<<"Edge collision p1: " << collPos.transpose() << "\n";
//                    std::cout<<"Edge collision p2: " << collPos2.transpose() << "\n";
                Eigen::Vector3d mMinus = collPos2 - collPos;
                collPos = P1Plus + sPlus * e1Plus;
                collPos2 = P + tPlus * e2;

//                    std::cout<<"After: " << collPos.transpose() << "\n";
//                    std::cout<<"Edge collision p1: " << collPos.transpose() << "\n";
//                    std::cout<<"Edge collision p2: " << collPos2.transpose() << "\n";
                Eigen::Vector3d mPlus = collPos2 - collPos;

                Eigen::Vector3d v0 = p0->vTemp;
                Eigen::Vector3d v1 = p1->vTemp;
                Eigen::Vector3d vColl = s * v0 + (1 - s) * v1;
                Eigen::Vector3d vCollN = vColl.dot(n) * n;
                Eigen::Vector3d vCollT = vColl - vCollN;
//                    return;
                if (mMinus.dot(mPlus) < 0) {
//                        std::cout<<"Edge collided!\n";
                    Eigen::Vector3d deltaVColl = (-vCollN * simParams.restitutionCoefficient) - vCollN;
                    Eigen::Vector3d deltaVCollPrime = deltaVColl / (s * s + (1 - s) * (1 - s));
                    Eigen::Vector3d deltaV0 = (1 - s) * deltaVCollPrime;
                    Eigen::Vector3d deltaV1 = (s) * deltaVCollPrime;
                    v0 += deltaV0;
                    v1 += deltaV1;

                    Eigen::Vector3d p0Offset = p0->x - collPos;
                    Eigen::Vector3d p1Offset = p1->x - collPos;

                    p0->v = v0;
                    p0->vTemp = v0;
                    p0->x = collPos2 + (n * 1e-3) + p0Offset;
                    p1->v = v1;
                    p1->vTemp = v1;
                    p1->x = collPos2 + (n * 1e-3) + p1Offset;

                    return;
                }
            }

            // QR
            e2 = R - Q;
            n = e1.cross(e2);
            q = Q - P1;
            n.normalize();
            s = (q.dot(e2.normalized().cross(n))) / (e1.dot(e2.normalized().cross(n)));
            t = (-q.dot(e1.normalized().cross(n))) / (e2.dot(e1.normalized().cross(n)));

            nPlus = e1Plus.cross(e2);
            nPlus.normalize();
            qPlus = Q - P1Plus;
            sPlus = (qPlus.dot(e2.normalized().cross(nPlus))) / (e1Plus.dot(e2.normalized().cross(nPlus)));
            tPlus = (-qPlus.dot(e1Plus.normalized().cross(nPlus))) / (e2.dot(e1Plus.normalized().cross(nPlus)));

            if ((s >= 0 && s <= 1 && t >= 0 && t <= 1) || (sPlus >= 0 && sPlus <= 1 && tPlus >= 0 && tPlus <= 1)) {
                Eigen::Vector3d collPos = P1 + s * e1;
                Eigen::Vector3d collPos2 = Q + t * e2;
                double dist = (collPos2 - collPos).norm();
//                    std::cout<<"Before: " << collPos.transpose() << "\n";
//                    std::cout<<"Edge collision p1: " << collPos.transpose() << "\n";
//                    std::cout<<"Edge collision p2: " << collPos2.transpose() << "\n";
                Eigen::Vector3d mMinus = collPos2 - collPos;
                collPos = P1Plus + sPlus * e1Plus;
                collPos2 = Q + tPlus * e2;

//                    std::cout<<"After: " << collPos.transpose() << "\n";
//                    std::cout<<"Edge collision p1: " << collPos.transpose() << "\n";
//                    std::cout<<"Edge collision p2: " << collPos2.transpose() << "\n";
                Eigen::Vector3d mPlus = collPos2 - collPos;
                Eigen::Vector3d v0 = p0->vTemp;
                Eigen::Vector3d v1 = p1->vTemp;
                Eigen::Vector3d vColl = s * v0 + (1 - s) * v1;
                Eigen::Vector3d vCollN = vColl.dot(n) * n;
                Eigen::Vector3d vCollT = vColl - vCollN;
//                    return;
                if (mMinus.dot(mPlus) < 0) {
//                        std::cout<<"Edge collided!\n";
                    Eigen::Vector3d deltaVColl = (-vCollN * simParams.restitutionCoefficient) - vCollN;
                    Eigen::Vector3d deltaVCollPrime = deltaVColl / (s * s + (1 - s) * (1 - s));
                    Eigen::Vector3d deltaV0 = (1 - s) * deltaVCollPrime;
                    Eigen::Vector3d deltaV1 = (s) * deltaVCollPrime;
                    v0 += deltaV0;
                    v1 += deltaV1;

                    Eigen::Vector3d p0Offset = p0->x - collPos;
                    Eigen::Vector3d p1Offset = p1->x - collPos;

                    p0->v = v0;
                    p0->vTemp = v0;
                    p0->x = collPos2 + (n * 1e-3) + p0Offset;
                    p1->v = v1;
                    p1->vTemp = v1;
                    p1->x = collPos2 + (n * 1e-3) + p1Offset;

                    return;
                }
            }

            // RP
            e2 = P - R;
            n = e1.cross(e2);
            q = R - P1;
            n.normalize();
            s = (q.dot(e2.normalized().cross(n))) / (e1.dot(e2.normalized().cross(n)));
            t = (-q.dot(e1.normalized().cross(n))) / (e2.dot(e1.normalized().cross(n)));

            nPlus = e1Plus.cross(e2);
            nPlus.normalize();
            qPlus = R - P1Plus;
            sPlus = (qPlus.dot(e2.normalized().cross(nPlus))) / (e1Plus.dot(e2.normalized().cross(nPlus)));
            tPlus = (-qPlus.dot(e1Plus.normalized().cross(nPlus))) / (e2.dot(e1Plus.normalized().cross(nPlus)));

            if ((s >= 0 && s <= 1 && t >= 0 && t <= 1) || (sPlus >= 0 && sPlus <= 1 && tPlus >= 0 && tPlus <= 1)) {
                Eigen::Vector3d collPos = P1 + s * e1;
                Eigen::Vector3d collPos2 = R + t * e2;
                double dist = (collPos2 - collPos).norm();
//                    std::cout<<"Before: \n";
//                    std::cout<<"Edge collision p1: " << collPos.transpose() << "\n";
//                    std::cout<<"Edge collision p2: " << collPos2.transpose() << "\n";
                Eigen::Vector3d mMinus = collPos2 - collPos;
                collPos = P1Plus + sPlus * e1Plus;
                collPos2 = R + tPlus * e2;

//                    std::cout<<"After: \n";
//                    std::cout<<"Edge collision p1: " << collPos.transpose() << "\n";
//                    std::cout<<"Edge collision p2: " << collPos2.transpose() << "\n";
                Eigen::Vector3d mPlus = collPos2 - collPos;
                Eigen::Vector3d v0 = p0->vTemp;
                Eigen::Vector3d v1 = p1->vTemp;
                Eigen::Vector3d vColl = s * v0 + (1 - s) * v1;
                Eigen::Vector3d vCollN = vColl.dot(n) * n;
                Eigen::Vector3d vCollT = vColl - vCollN;
//                    return;
                if (mMinus.dot(mPlus) < 0) {
//                        std::cout<<"Edge collided!\n";
                    Eigen::Vector3d deltaVColl = (-vCollN * simParams.restitutionCoefficient) - vCollN;
                    Eigen::Vector3d deltaVCollPrime = deltaVColl / (s * s + (1 - s) * (1 - s));
                    Eigen::Vector3d deltaV0 = (1 - s) * deltaVCollPrime;
                    Eigen::Vector3d deltaV1 = (s) * deltaVCollPrime;
                    v0 += deltaV0;
                    v1 += deltaV1;

                    Eigen::Vector3d p0Offset = p0->x - collPos;
                    Eigen::Vector3d p1Offset = p1->x - collPos;

                    p0->v = v0;
                    p0->vTemp = v0;
                    p0->x = collPos2 + (n * 1e-3) + p0Offset;
                    p1->v = v1;
                    p1->vTemp = v1;
                    p1->x = collPos2 + (n * 1e-3) + p1Offset;

                    return;
                }
            }

        }
    }
}

void SingleSpring::detectCollision(std::shared_ptr<Particle> particle, std::vector<std::shared_ptr<Shape> >& shapes, SimParams& simParams) {
    if (particle->fixed) {
        return;
    }
    Eigen::Vector3d x = particle->xTemp;
    Eigen::Vector3d xNew = particle->x;
    Eigen::Vector3d vel = particle->vTemp;

    if (vel.norm() <= 0.08) { // v close to zero
        if (particle->hasCollided && (xNew - particle->xc).norm() <= 1e-2) { // position on surface
//            if (particle->f.dot(particle->nc) < 0.0) { // check this
                particle->x = x;
                particle->v = Eigen::Vector3d(0.0, 0.0, 0.0);
                return;
//            }
        }
    }

    for (auto shape: shapes) {
        if (!shape->getObstacle()) {
            continue;
        }
        for (Polygon p : shape->getPolygons()) {
            Eigen::Vector3d P = p.points[0];
            Eigen::Vector3d Q = p.points[1];
            Eigen::Vector3d R = p.points[2];

            Eigen::Vector3d u = Q - P;
            Eigen::Vector3d v = R - P;
            Eigen::Vector3d n = u.cross(v);
            n.normalize();
            if (n.norm() == 0) continue;

            double pn = P.dot(n);
            double d0 = (n.dot(x) - pn);
            double d1 = (n.dot(xNew) - pn);

            if (sgn(d0) * sgn(d1) >= 0) continue;

            Eigen::Vector3d dir = xNew - x;
            dir.normalize();
            double t = (pn - n.dot(x))/(n.dot(dir));
            Eigen::Vector3d xColl = x + (t * dir);

            // check if point is inside polygon by projecting along axis with highest normal value
            Eigen::Vector3d Pproj, Qproj, Rproj, xCollProj;
            if (std::abs(n.x()) >= std::max(std::abs(n.y()), std::abs(n.z()))) {
                Pproj = Eigen::Vector3d(P.y(), P.z(), 0.0);
                Qproj = Eigen::Vector3d(Q.y(), Q.z(), 0.0);
                Rproj = Eigen::Vector3d(R.y(), R.z(), 0.0);
                xCollProj = Eigen::Vector3d(xColl.y(), xColl.z(), 0.0);
            } else if (std::abs(n.y()) >= std::max(std::abs(n.x()), std::abs(n.z()))) {
                Pproj = Eigen::Vector3d(P.z(), P.x(), 0.0);
                Qproj = Eigen::Vector3d(Q.z(), Q.x(), 0.0);
                Rproj = Eigen::Vector3d(R.z(), R.x(), 0.0);
                xCollProj = Eigen::Vector3d(xColl.z(), xColl.x(), 0.0);
            } else {
                Pproj = Eigen::Vector3d(P.x(), P.y(), 0.0);
                Qproj = Eigen::Vector3d(Q.x(), Q.y(), 0.0);
                Rproj = Eigen::Vector3d(R.x(), R.y(), 0.0);
                xCollProj = Eigen::Vector3d(xColl.x(), xColl.y(), 0.0);
            }

            Eigen::Vector3d edge1Proj = (Pproj - Qproj);
            Eigen::Vector3d edge2Proj = (Pproj - Rproj);
            Eigen::Vector3d areaProj = edge1Proj.cross(edge2Proj);
            double S = 0.5 * areaProj.norm() * (areaProj.z() / std::abs(areaProj.z()));
            Eigen::Vector3d AaVec = (Qproj - xCollProj).cross(Rproj - xCollProj);
            double Aa = 0.5 * AaVec.norm() * (AaVec.z()/std::abs(AaVec.z()));
            Eigen::Vector3d AbVec = (Rproj - xCollProj).cross(Pproj - xCollProj);
            double Ab = 0.5 * AbVec.norm() * (AbVec.z()/std::abs(AbVec.z()));
            Eigen::Vector3d AcVec = (Pproj - xCollProj).cross(Qproj - xCollProj);
            double Ac = 0.5 * AcVec.norm() * (AcVec.z()/std::abs(AcVec.z()));
            double a = Aa/S;
            double b = Ab/S;
            double c = Ac/S;

            if (a >= 0 && a <= 1.0 && b >= 0 && b <= 1 && c >= 0 && c <= 1) { // collision
                Eigen::Vector3d xc = xColl;
                Eigen::Vector3d nc = n;
                xNew = xc + 1e-3 * nc;
                Eigen::Vector3d vn = vel.dot(nc) * nc;
                Eigen::Vector3d vt = vel - vn;
                vn *= -simParams.restitutionCoefficient;
                vel = vt + vn;
//                double dColl = -(xNew - p.points[0]).dot(nc) * nc;
//                std::cout<<"Collided: dColl:" <<dColl.transpose()<<"\n";
//                std::cout<<"xColl:" <<xColl.transpose()<<"\n";
//                std::cout<<"pts 0:" <<p.points[0].transpose()<<"\n";
//                std::cout<<"pts 1:" <<p.points[1].transpose()<<"\n";
//                std::cout<<"pts 2:" <<p.points[2].transpose()<<"\n";
//                std::cout<<"Aa :" <<Aa<<"\n";
//                std::cout<<"Ab :" <<Ab<<"\n";
//                std::cout<<"Ac :" <<Ac<<"\n";
                particle->didCollide = true;
                particle->hasCollided = true;
                particle->nc = nc;
                particle->xc = xc;
                particle->x = xNew;
                particle->xTemp = xNew;
                particle->v = vel;
                particle->vTemp = vel;
                std::cout<<"Collided\n";
                return;
            } else {
                continue;
            }
        }
    }
}

void SingleSpring::init() {
    glGenVertexArrays(1, &VAO);
    glBindVertexArray(VAO);

    glGenBuffers(1, &posBufID);
    glGenBuffers(1, &eleBufID);

    // Send the position array to the GPU
    glBindBuffer(GL_ARRAY_BUFFER, posBufID);
    glBufferData(GL_ARRAY_BUFFER, posBuf.size()*sizeof(float), &posBuf[0], GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, eleBufID);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, eleBuf.size()*sizeof(unsigned int), &eleBuf[0], GL_STATIC_DRAW);
    // Unbind the arrays
//    glBindBuffer(GL_ARRAY_BUFFER, 0);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);
    glBindVertexArray(0);

    GLSL::checkError(GET_FILE_LINE);
}

void SingleSpring::cleanupBuffers() {
    glDeleteBuffers(1, &posBufID);
    glDeleteBuffers(1, &eleBufID);
    glDeleteVertexArrays(1, &VAO);
}

void SingleSpring::draw(const std::shared_ptr<Program> prog) const {
    GLSL::checkError(GET_FILE_LINE);
    glBindVertexArray(VAO);
    // Bind position buffer
    int h_pos = prog->getAttribute("aPos");
    glEnableVertexAttribArray(h_pos);
    GLSL::checkError(GET_FILE_LINE);
    glBindBuffer(GL_ARRAY_BUFFER, posBufID);
    glBufferData(GL_ARRAY_BUFFER, posBuf.size()*sizeof(float), &posBuf[0], GL_DYNAMIC_DRAW);
    glVertexAttribPointer(h_pos, 3, GL_FLOAT, GL_FALSE, 0, (const void *)0);
    glUniform3f(prog->getUniform("color"), 0.0f, 0.0f, 1.0f);
    glPointSize(5.0);
    int count = (int)posBuf.size()/3; // number of indices to be rendered
    glDrawArrays(GL_POINTS , 0, count);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, eleBufID);
    GLSL::checkError(GET_FILE_LINE);
    glUniform3f(prog->getUniform("color"), 0.8f, 0.8f, 0.8f);
    GLSL::checkError(GET_FILE_LINE);

    glDrawElements(GL_LINES, eleBuf.size(), GL_UNSIGNED_INT, (const void *)0);
    glDisableVertexAttribArray(h_pos);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
    GLSL::checkError(GET_FILE_LINE);
}