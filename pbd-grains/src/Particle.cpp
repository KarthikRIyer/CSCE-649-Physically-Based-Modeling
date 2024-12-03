#include <iostream>

#define GLEW_STATIC
#include <GL/glew.h>
#include <GLFW/glfw3.h>

#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "Particle.h"
#include "Shape.h"
#include "Program.h"
#include "MatrixStack.h"
#include "IForceField.h"
#include "SimParams.h"
#include "Polygon.h"

using namespace std;

Particle::Particle() :
	r(1.0),
	m(1.0),
	i(-1),
	x(0.0, 0.0, 0.0),
	v(0.0, 0.0, 0.0),
	fixed(true)
{
    if (m != 0.0)
        invM = 1/m;
}

Particle::Particle(const shared_ptr<Shape> s, bool drawSphere) :
	r(1.0),
	m(1.0),
	i(-1),
	x(0.0, 0.0, 0.0),
	xc(0.0, 0.0, 0.0),
	nc(0.0, 0.0, 0.0),
	v(0.0, 0.0, 0.0),
	fixed(true),
    didCollide(false),
    hasCollided(false),
	sphere(s),
	drawSphere(drawSphere)
{
    if (m != 0.0)
        invM = 1/m;
}

Particle::~Particle()
{
}

void Particle::tare()
{
	x0 = x;
	v0 = v;
}

void Particle::reset()
{
	x = x0;
	v = v0;
}

double sgn(double x) {
    if (x > 0) return 1;
    if (x < 0) return -1;
    return 0;
}

double Particle::detectCollision(double h, std::vector<std::shared_ptr<Shape> >& shapes) {
//    Eigen::Vector3d xNew = x + (v * h);
    didCollide = false;
    for (auto shape: shapes) {
        for (Polygon p : shape->getPolygons()) {
            Eigen::Vector3d u = p.points[1] - p.points[0];
            Eigen::Vector3d v = p.points[2] - p.points[0];
            Eigen::Vector3d n = u.cross(v);
            n.normalize();
            if (n.norm() == 0) continue;

            // project center of sphere to triangle plane
            Eigen::Vector3d dir = xTemp - p.points[0];
            double dist = dir.dot(n);
            Eigen::Vector3d xp = xTemp - (dist * n);

            Eigen::Vector3d P = p.points[0];
            Eigen::Vector3d Q = p.points[1];
            Eigen::Vector3d R = p.points[2];
            Eigen::Vector3d Pproj, Qproj, Rproj, xCollProj;
            if (std::abs(n.x()) >= std::max(std::abs(n.y()), std::abs(n.z()))) {
                Pproj = Eigen::Vector3d(P.y(), P.z(), 0.0);
                Qproj = Eigen::Vector3d(Q.y(), Q.z(), 0.0);
                Rproj = Eigen::Vector3d(R.y(), R.z(), 0.0);
                xCollProj = Eigen::Vector3d(xp.y(), xp.z(), 0.0);
            } else if (std::abs(n.y()) >= std::max(std::abs(n.x()), std::abs(n.z()))) {
                Pproj = Eigen::Vector3d(P.z(), P.x(), 0.0);
                Qproj = Eigen::Vector3d(Q.z(), Q.x(), 0.0);
                Rproj = Eigen::Vector3d(R.z(), R.x(), 0.0);
                xCollProj = Eigen::Vector3d(xp.z(), xp.x(), 0.0);
            } else {
                Pproj = Eigen::Vector3d(P.x(), P.y(), 0.0);
                Qproj = Eigen::Vector3d(Q.x(), Q.y(), 0.0);
                Rproj = Eigen::Vector3d(R.x(), R.y(), 0.0);
                xCollProj = Eigen::Vector3d(xp.x(), xp.y(), 0.0);
            }

            // check if point is inside polygon
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

            if (std::abs(dist) < r && a >= 0 && a <= 1.0 && b >= 0 && b <= 1 && c >= 0 && c <= 1) { // collision
                didCollide = true;
                hasCollided = true;
                xc = xp;
                nc = n;
//                std::cout<<"nc: "<<nc.transpose()<<"\n";
//                std::cout<<"xc: "<<xc.transpose()<<"\n";
//                std::cout<<"x: "<<x.transpose()<<"\n";
//                std::cout<<"dist: "<<dist<<"\n";
//                double frac = d0 / (d0 - d1);
                double frac = dist / r;
                return frac;
            } else if (std::abs(dist) < r && (xp - p.points[0]).norm() < r) {
                // center of the sphere is outside the triangle but the
                // sphere still intersects a corner of the triangle
                didCollide = true;
                hasCollided = true;
                xc = p.points[0];
                nc = (xTemp - xc).normalized();
                double frac = (xTemp - xc).norm() / r;
                return frac;
            } else if (std::abs(dist) < r && (xp - p.points[1]).norm() < r) {
                // center of the sphere is outside the triangle but the
                // sphere still intersects a corner of the triangle
                didCollide = true;
                hasCollided = true;
                xc = p.points[1];
                nc = (xTemp - xc).normalized();
                double frac = (xTemp - xc).norm() / r;
                return frac;
            } else if (std::abs(dist) < r && (xp - p.points[2]).norm() < r) {
                // center of the sphere is outside the triangle but the
                // sphere still intersects a corner of the triangle
                didCollide = true;
                hasCollided = true;
                xc = p.points[2];
                nc = (xTemp - xc).normalized();
                double frac = (xTemp - xc).norm() / r;
                return frac;
            }
//            else if (std::abs(dist) < r) {
//                // center of sphere is close to edges of triangle
//                Eigen::Vector3d A = p.points[0];
//                Eigen::Vector3d B = p.points[1];
//                Eigen::Vector3d C = xTemp;
//
//                Eigen::Vector3d AB = (B - A);
//                Eigen::Vector3d AC = (C - A);
//                Eigen::Vector3d AD = AB * (AB.dot(AC))/(AB.dot(AB));
//                Eigen::Vector3d D = A + AD;
//                if (AD.norm() <= AB.norm() && AD.dot(AB) > 0) {
//                    didCollide = true;
//                    hasCollided = true;
//                    xc = D; // collision point is projection of center on edge
//                    nc = (xTemp - D).normalized(); // normal is from collision point to center of sphere
//                    double frac = (xTemp - xc).norm() / r;
//                    return frac;
//                }
//
//                A = p.points[1];
//                B = p.points[2];
//                C = xTemp;
//                AB = (B - A);
//                AC = (C - A);
//                AD = AB * (AB.dot(AC))/(AB.dot(AB));
//                D = A + AD;
//                if (AD.norm() <= AB.norm() && AD.dot(AB) > 0) {
//                    didCollide = true;
//                    hasCollided = true;
//                    xc = D; // collision point is projection of center on edge
//                    nc = (xTemp - D).normalized(); // normal is from collision point to center of sphere
//                    double frac = (xTemp - xc).norm() / r;
//                    return frac;
//                }
//
//                A = p.points[2];
//                B = p.points[0];
//                C = xTemp;
//                AB = (B - A);
//                AC = (C - A);
//                AD = AB * (AB.dot(AC))/(AB.dot(AB));
//                D = A + AD;
//                if (AD.norm() <= AB.norm() && AD.dot(AB) > 0) {
//                    didCollide = true;
//                    hasCollided = true;
//                    xc = D; // collision point is projection of center on edge
//                    nc = (xTemp - D).normalized(); // normal is from collision point to center of sphere
//                    double frac = (xTemp - xc).norm() / r;
//                    return frac;
//                }
//
//                continue;
//            }
            else {
                continue;
            }
        }
    }
    return 1.0;
}

void Particle::step(double h, std::vector<std::shared_ptr<IForceField>>& forceFields, SimParams& simParams) {

    Eigen::Vector3d vNew = v;
    Eigen::Vector3d fNet(0, 0, 0);
    for (auto forceField: forceFields) {
        fNet += forceField->getForce(x);
        vNew += (forceField->getForce(x) * h);
    }
    vNew -= (h * (simParams.airFrictionFactor / m) * v);
//    std::cout<<"V: "<<v.norm()<<"\n";
    if (v.norm() <= 0.08) { // if v is close to zero
//        std::cout<<"dist: "<<(x - xc).norm()<<"\n";
        if (hasCollided && (x - (r * nc) - xc).norm() <= 1e-2) { // position is on surface
//            std::cout<<"fNet.dot(nc): "<<fNet.dot(nc)<<"\n";
            if (fNet.dot(nc) < 0.0) { // acc towards the surface
                Eigen::Vector3d an = fNet.dot(nc) * nc.normalized();
                Eigen::Vector3d at = fNet - an;
                if (simParams.frictionCoeff * an.norm() >= at.norm()) { // friction must overcome tangential acceleration
//                    std::cout<<"Stopped\n";
                    return;
                }
            }
        }
    }

    Eigen::Vector3d xNew = x + (v * h);

    x = xNew;
    v = vNew;

    if (didCollide) {
        didCollide = false;
        nc.normalize();
        x = xc + (nc * r);
        x += (1e-7 * nc); // to handle precision errors
        Eigen::Vector3d vn = v.dot(nc) * nc;
        Eigen::Vector3d vt = v - vn;
//        std::cout<<"v: "<< v.transpose()<<"\n";
//        std::cout<<"vt: "<< vt.transpose()<<"\n";
//        std::cout<<"vn: "<< vn.transpose()<<"\n";
//        std::cout<<"nc: "<< nc.transpose()<<"\n";
//        std::cout<<"restitution coeff: "<< simParams.restitutionCoeff<<"\n";
        vn *= -simParams.restitutionCoeff;

        double fric = simParams.frictionCoeff * (fNet.dot(nc));
        Eigen::Vector3d aFric = std::min(fric, (vt / h).norm()) * vt.normalized();
//        std::cout<<"aFric: "<< aFric<<"\n";
//        std::cout<<"vt after: "<< vt.transpose()<<"\n";
//        std::cout<<"vn after: "<< vn.transpose()<<"\n";
        vt += (aFric * h);

        v = vn + vt;
//        std::cout<<"v after: "<< v.transpose()<<"\n";
    }
}

void Particle::draw(shared_ptr<MatrixStack> MV, const shared_ptr<Program> prog) const
{
	if(drawSphere) {
		MV->pushMatrix();
		MV->translate(x(0), x(1), x(2));
		MV->scale(r);
		glUniformMatrix4fv(prog->getUniform("MV"), 1, GL_FALSE, glm::value_ptr(MV->topMatrix()));
		sphere->setProgram(prog);
		sphere->draw();
		MV->popMatrix();
	}
}
