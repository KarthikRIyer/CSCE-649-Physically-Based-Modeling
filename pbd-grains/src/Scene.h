#pragma once
#ifndef Scene_H
#define Scene_H

#include <vector>
#include <memory>
#include <string>
#include <map>

#define EIGEN_DONT_ALIGN_STATICALLY
#include <Eigen/Dense>
#include "SimParams.h"

class Cloth;
class Particle;
class MatrixStack;
class Program;
class Shape;
class Texture;
class IForceField;
class Gravity;
class Wind;
class Grains;

class Scene
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	Scene();
	virtual ~Scene();

	void load(const std::string &RESOURCE_DIR, const std::string &DATA_DIR, int texUnit);
	void init();
	void cleanup();
	void tare();
	void reset();
	void updateSimParams(SimParams& simParams);
	void step(std::ofstream &outputFile, bool writeToFile);
	void setSceneNum(int sceneNum);
	
	void draw(std::shared_ptr<MatrixStack> MV, const std::shared_ptr<Program> prog) const;

	double getTime() const { return t; }
	double getTimestep() const { return h; }
	
private:
	double t;
	double h;
	SimParams simParams;

	std::shared_ptr<Gravity> gravity;
	std::shared_ptr<Wind> wind;
	std::vector<std::shared_ptr<IForceField>> forceFields;
    std::shared_ptr<Texture> sphereTexture;

	std::shared_ptr<Shape> sphereShape;
	std::vector< std::shared_ptr<Particle> > spheres;

    std::shared_ptr<Grains> grains;

    std::vector<std::shared_ptr<Shape> > shapes;
    std::map<std::string, std::shared_ptr<Texture>> textureMap;

    std::vector<std::string> textureData;
    std::vector<std::vector<std::string>> meshData;
    std::vector<std::string> grainsData;
    std::vector<std::string> staticGrainsData;
    int sceneIndex = 0;

    void loadDataInputFile(const std::string &DATA_DIR, const std::string &FILE_NAME);
    std::vector<Eigen::Vector3d> readGrainsFile(const std::string& filename);
};

#endif
