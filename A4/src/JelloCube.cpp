//
// Created by Karthik Iyer on 21/10/24.
//

#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>

#include <iostream>

#include "JelloCube.h"
#include "Particle.h"
#include "Spring.h"
#include "GLSL.h"
#include "Program.h"
#include "IForceField.h"

JelloCube::JelloCube() {
    std::shared_ptr<Particle> p0 = std::make_shared<Particle>();
    std::shared_ptr<Particle> p1 = std::make_shared<Particle>();
//    std::shared_ptr<Particle> p2 = std::make_shared<Particle>();
//    std::shared_ptr<Particle> p3 = std::make_shared<Particle>();
//    std::shared_ptr<Particle> p4 = std::make_shared<Particle>();
//    std::shared_ptr<Particle> p5 = std::make_shared<Particle>();
//    std::shared_ptr<Particle> p6 = std::make_shared<Particle>();
//    std::shared_ptr<Particle> p7 = std::make_shared<Particle>();

    p0->x0 = Eigen::Vector3d(-1.0, 1.0, 0.0);
    p1->x0 = Eigen::Vector3d(1.0, 1.0, 0.0);
//    p2->x0 = Eigen::Vector3d(1.0, 2.0, 0.0);
//    p3->x0 = Eigen::Vector3d(-1.0, 2.0, 0.0);
//    p4->x0 = Eigen::Vector3d(-1.0, 2.0, 1.0);
//    p5->x0 = Eigen::Vector3d(1.0, 2.0, 1.0);
//    p6->x0 = Eigen::Vector3d(1.0, 1.0, 1.0);
//    p7->x0 = Eigen::Vector3d(-1.0, 1.0, 1.0);

    particles.push_back(p0);
    particles.push_back(p1);
//    particles.push_back(p2);
//    particles.push_back(p3);
//    particles.push_back(p4);
//    particles.push_back(p5);
//    particles.push_back(p6);
//    particles.push_back(p7);

    for (int i = 0; i < particles.size(); ++i) {
        particles[i]->v0 = Eigen::Vector3d(0.0, 0.0, 0.0);
        particles[i]->v = particles[i]->v0;
        particles[i]->x = particles[i]->x0;
        particles[i]->f = Eigen::Vector3d(0.0, 0.0, 0.0);
    }

    std::shared_ptr<Spring> s0 = std::make_shared<Spring>(p0, p1, 15.0, 0.5);
//    std::shared_ptr<Spring> s1 = std::make_shared<Spring>(p1, p2, 15.0, 0.5);
//    std::shared_ptr<Spring> s2 = std::make_shared<Spring>(p2, p3, 15.0, 0.5);
//    std::shared_ptr<Spring> s3 = std::make_shared<Spring>(p3, p0, 15.0, 0.5);
//
//    std::shared_ptr<Spring> s4 = std::make_shared<Spring>(p7, p6, 15.0, 0.5);
//    std::shared_ptr<Spring> s5 = std::make_shared<Spring>(p6, p5, 15.0, 0.5);
//    std::shared_ptr<Spring> s6 = std::make_shared<Spring>(p5, p4, 15.0, 0.5);
//    std::shared_ptr<Spring> s7 = std::make_shared<Spring>(p4, p7, 15.0, 0.5);
//
//    std::shared_ptr<Spring> s8 = std::make_shared<Spring>(p0, p7, 15.0, 0.5);
//    std::shared_ptr<Spring> s9 = std::make_shared<Spring>(p1, p6, 15.0, 0.5);
//    std::shared_ptr<Spring> s10 = std::make_shared<Spring>(p3, p4, 15.0, 0.5);
//    std::shared_ptr<Spring> s11 = std::make_shared<Spring>(p2, p5, 15.0, 0.5);

    // cross springs
//    std::shared_ptr<Spring> s12 = std::make_shared<Spring>(p1, p3, 15.0, 0.5);
//    std::shared_ptr<Spring> s13 = std::make_shared<Spring>(p0, p2, 15.0, 0.5);
//
//    std::shared_ptr<Spring> s14 = std::make_shared<Spring>(p4, p6, 15.0, 0.5);
//    std::shared_ptr<Spring> s15 = std::make_shared<Spring>(p7, p5, 15.0, 0.5);
//
//    std::shared_ptr<Spring> s16 = std::make_shared<Spring>(p4, p2, 15.0, 0.5);
//    std::shared_ptr<Spring> s17 = std::make_shared<Spring>(p3, p5, 15.0, 0.5);
//
//    std::shared_ptr<Spring> s18 = std::make_shared<Spring>(p7, p1, 15.0, 0.5);
//    std::shared_ptr<Spring> s19 = std::make_shared<Spring>(p6, p0, 15.0, 0.5);
//
//    std::shared_ptr<Spring> s20 = std::make_shared<Spring>(p3, p7, 15.0, 0.5);
//    std::shared_ptr<Spring> s21 = std::make_shared<Spring>(p0, p4, 15.0, 0.5);
//
//    std::shared_ptr<Spring> s22 = std::make_shared<Spring>(p2, p6, 15.0, 0.5);
//    std::shared_ptr<Spring> s23 = std::make_shared<Spring>(p1, p5, 15.0, 0.5);
//
//    std::shared_ptr<Spring> s24 = std::make_shared<Spring>(p0, p5, 15.0, 0.5);
//    std::shared_ptr<Spring> s25 = std::make_shared<Spring>(p1, p4, 15.0, 0.5);
//    std::shared_ptr<Spring> s26 = std::make_shared<Spring>(p6, p3, 15.0, 0.5);
//    std::shared_ptr<Spring> s27 = std::make_shared<Spring>(p7, p2, 15.0, 0.5);

    springs.push_back(s0);
//    springs.push_back(s1);
//    springs.push_back(s2);
//    springs.push_back(s3);
//    springs.push_back(s4);
//    springs.push_back(s5);
//    springs.push_back(s6);
//    springs.push_back(s7);
//    springs.push_back(s8);
//    springs.push_back(s9);
//    springs.push_back(s10);
//    springs.push_back(s11);

//    springs.push_back(s11);
//    springs.push_back(s12);
//    springs.push_back(s13);
//    springs.push_back(s14);
//    springs.push_back(s15);
//    springs.push_back(s16);
//    springs.push_back(s17);
//    springs.push_back(s18);
//    springs.push_back(s19);
//    springs.push_back(s20);
//    springs.push_back(s21);
//    springs.push_back(s22);
//    springs.push_back(s23);

//    springs.push_back(s24);
//    springs.push_back(s25);
//    springs.push_back(s26);
//    springs.push_back(s27);

    for (int i = 0; i < springs.size(); ++i) {
        springs[i]->d = 0;
    }
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

    p0->x = Eigen::Vector3d(-0.8, 1.0, 0.0);
//    p1->x = Eigen::Vector3d(0.5, 1.0, 0.0);
}

JelloCube::~JelloCube() = default;

void JelloCube::tare() {
    for (const auto & particle : particles) {
        particle->tare();
    }
}

void JelloCube::reset() {
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

std::pair<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d >> JelloCube::getVelAcc(double h, std::vector<std::shared_ptr<IForceField>>& forceFields) {
    int integrationScheme = 0;
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

void JelloCube::step(double h, std::vector<std::shared_ptr<IForceField>>& forceFields, SimParams& simParams) {
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
        particles[i]->x += va.first[i] * h;
        particles[i]->v += va.second[i] * h;
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

void JelloCube::init() {
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

void JelloCube::cleanupBuffers() {
    glDeleteBuffers(1, &posBufID);
    glDeleteBuffers(1, &eleBufID);
    glDeleteVertexArrays(1, &VAO);
}

void JelloCube::draw(const std::shared_ptr<Program> prog) const {
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