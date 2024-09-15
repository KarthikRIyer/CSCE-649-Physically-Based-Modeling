#pragma once
#ifndef Particle_H
#define Particle_H

#include <vector>
#include <memory>

#define EIGEN_DONT_ALIGN_STATICALLY
#include <Eigen/Dense>

class Shape;
class Program;
class MatrixStack;
class IForceField;
struct SimParams;

class Particle
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	Particle();
	Particle(const std::shared_ptr<Shape> shape, bool drawSphere = false);
	virtual ~Particle();
	void tare();
	void reset();
	double detectCollision(double h, std::vector<std::shared_ptr<Shape> >& shapes);
	void step(double h, std::vector<std::shared_ptr<IForceField>>& forceFields, SimParams& simParams);
	void draw(std::shared_ptr<MatrixStack> MV, const std::shared_ptr<Program> p) const;
	
	double r; // radius
	double m; // mass
	int i;  // starting index
	Eigen::Vector3d x0; // initial position
	Eigen::Vector3d v0; // initial velocity
	Eigen::Vector3d x;  // position
//	Eigen::Vector3d xc;  // collision pos
	Eigen::Vector3d nc;  // collision nor
	Eigen::Vector3d dColl;  // collision vec
	double collDist;
	Eigen::Vector3d v;  // velocity
	Eigen::Vector3d f;  // force
	bool fixed;
	bool didCollide;
	bool hasCollided;

private:
	const std::shared_ptr<Shape> sphere;
	bool drawSphere = true;
};

#endif
