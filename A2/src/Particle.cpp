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
                didCollide = true;
                hasCollided = true;
                xc = xColl;
                nc = n;
                dColl = -(xNew - p.points[0]).dot(nc) * nc;
//                std::cout<<"Collided: dColl:" <<dColl.transpose()<<"\n";
//                std::cout<<"xColl:" <<xColl.transpose()<<"\n";
//                std::cout<<"pts 0:" <<p.points[0].transpose()<<"\n";
//                std::cout<<"pts 1:" <<p.points[1].transpose()<<"\n";
//                std::cout<<"pts 2:" <<p.points[2].transpose()<<"\n";
//                std::cout<<"Aa :" <<Aa<<"\n";
//                std::cout<<"Ab :" <<Ab<<"\n";
//                std::cout<<"Ac :" <<Ac<<"\n";
                collDist = std::abs(d1);
                return 1.0;
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
    vNew -= (h * (simParams.airFrictionFactor / m) * v);
//    std::cout<<"V: "<<v.norm()<<"\n";
    if (v.norm() <= 0.08) { // if v is close to zero
//        std::cout<<"dist: "<<(x - xc).norm()<<"\n";
        if (hasCollided && ((x - xc).norm() <= 1e-2)) { // position is on surface
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
//        x = xc + (nc * r);
        x = x + (1.0 + simParams.restitutionCoeff) * (dColl);
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
