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
    Eigen::Vector3d xNew = x + (v * h);
    didCollide = false;
    for (auto shape: shapes) {
        for (Polygon p : shape->getPolygons()) {
            Eigen::Vector3d u = p.points[1] - p.points[0];
            Eigen::Vector3d v = p.points[2] - p.points[0];
            Eigen::Vector3d n = u.cross(v);
            n.normalize();
            if (n.norm() == 0) continue;

            double pn = p.points[0].dot(n);
            double d0 = (n.x() * x.x() + n.y() * x.y() + n.z() * x.z() - pn);
            double d1 = (n.x() * xNew.x() + n.y() * xNew.y() + n.z() * xNew.z() - pn);

            if (sgn(d0) * sgn(d1) >= 0) continue;

            Eigen::Vector3d dir = xNew - x;
            dir.normalize();
            double t = (pn - n.dot(x))/(n.dot(dir));
            Eigen::Vector3d xColl = x + (t * dir);

            // check if point is inside polygon
            double S = 0.5 * u.cross(v).norm();
            double Aa = 0.5 * (p.points[1] - xColl).cross(p.points[2] - xColl).norm();
            double Ab = 0.5 * (p.points[2] - xColl).cross(p.points[0] - xColl).norm();
            double Ac = 0.5 * (p.points[0] - xColl).cross(p.points[1] - xColl).norm();
            double a = Aa/S;
            double b = Ab/S;
            double c = Ac/S;

            if (a >= 0 && a <= 1.0 && b >= 0 && b <= 1 && c >= 0 && c <= 1) { // collision
                didCollide = true;
                hasCollided = true;
                xc = xColl;
                nc = n;
                double frac = d0 / (d0 - d1);
                return frac;
            } else {
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
    vNew -= ((simParams.airFrictionFactor / m) * v);
    std::cout<<"V: "<<v.norm()<<"\n";
    if (v.norm() <= 0.08) { // if v is close to zero
        std::cout<<"dist: "<<(x - xc).norm()<<"\n";
        if (hasCollided && (x - xc).norm() <= 2e-2) { // position is on surface
//            std::cout<<"fNet.dot(nc): "<<fNet.dot(nc)<<"\n";
            if (fNet.dot(nc) < 0.0) { // acc towards the surface
                Eigen::Vector3d an = fNet.dot(nc) * nc.normalized();
                Eigen::Vector3d at = fNet - an;
                if (simParams.frictionCoeff * an.norm() >= at.norm()) { // friction must overcome tangential acceleration
                    std::cout<<"Stopped\n";
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
        x = xc;
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
