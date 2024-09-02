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
	v(0.0, 0.0, 0.0),
	fixed(true),
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

void Particle::step(double h, std::vector<std::shared_ptr<IForceField>>& forceFields, SimParams& simParams) {
    Eigen::Vector3d vNew = v;
    for (auto forceField: forceFields) {
        vNew += (forceField->getForce(x) * h);
    }
    vNew -= ((simParams.airFrictionFactor / m) * v);

    Eigen::Vector3d xNew = x + (v * h);

    x = xNew;
    v = vNew;
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
