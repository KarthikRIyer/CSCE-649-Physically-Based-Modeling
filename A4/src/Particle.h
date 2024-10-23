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
	Eigen::Vector3d xc;  // collision pos
	Eigen::Vector3d nc;  // collision nor
	Eigen::Vector3d dColl;  // collision vec
	double collDist;
	Eigen::Vector3d v;  // velocity
	Eigen::Vector3d f;  // force
	Eigen::Vector3d fExt;  // force

    Eigen::Vector3d xTemp;
    Eigen::Vector3d vTemp;

    Eigen::Vector3d vk1;
    Eigen::Vector3d vk2;
    Eigen::Vector3d vk3;
    Eigen::Vector3d vk4;

    Eigen::Vector3d ak1;
    Eigen::Vector3d ak2;
    Eigen::Vector3d ak3;
    Eigen::Vector3d ak4;

	bool fixed;
	bool didCollide;
	bool hasCollided;

	double tStart;
	double tEnd;

private:
	const std::shared_ptr<Shape> sphere;
	bool drawSphere = true;
};

#endif
