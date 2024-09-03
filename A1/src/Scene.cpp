#include <iostream>
#include <fstream>
#include <glm/gtc/type_ptr.hpp>

#include "Scene.h"
#include "Particle.h"
#include "Shape.h"
#include "Program.h"
#include "Texture.h"
#include "MatrixStack.h"
#include "Gravity.h"
#include "Wind.h"

using namespace std;
using namespace Eigen;

void Scene::loadDataInputFile(const std::string &DATA_DIR)
{
    string filename = DATA_DIR + "input.txt";
    ifstream in;
    in.open(filename);
    if(!in.good()) {
        cout << "Cannot read " << filename << endl;
        return;
    }
    cout << "Loading " << filename << endl;

    string line;
    while(1) {
        getline(in, line);
        if(in.eof()) {
            break;
        }
        if(line.empty()) {
            continue;
        }
        // Skip comments
        if(line.at(0) == '#') {
            continue;
        }
        // Parse lines
        string key, value;
        stringstream ss(line);
        // key
        ss >> key;
        if(key.compare("TEXTURE") == 0) {
            ss >> value;
            textureData.push_back(value);
        } else if(key.compare("MESH") == 0) {
            vector<string> mesh;
            ss >> value;
            mesh.push_back(value); // obj
            ss >> value;
            mesh.push_back(value); // texture
            meshData.push_back(mesh);
        } else {
            cout << "Unknown key word: " << key << endl;
        }
    }
    in.close();
}

Scene::Scene() :
	t(0.0),
	h(5e-3)
{
}

Scene::~Scene()
{
}

void Scene::load(const string &RESOURCE_DIR, const string &DATA_DIR, int texUnit)
{
	// Units: meters, kilograms, seconds
	h = 5e-3;

	gravity = std::make_shared<Gravity>(Eigen::Vector3d(0.0, -9.8, 0.0));
	wind = std::make_shared<Wind>(5.0, Eigen::Vector3d(1.0, 0.0, 0.0));
	forceFields.push_back(gravity);
//	forceFields.push_back(wind);

	if (sceneIndex == 0) {
        sphereShape = make_shared<Shape>();
        sphereShape->loadMesh(RESOURCE_DIR + "sphere2.obj");

        auto sphere = make_shared<Particle>(sphereShape, true);
        spheres.push_back(sphere);
        sphere->r = 0.13;
        sphere->x = Vector3d(0.0, 1.0, 0.0);
        sphere->x0 = Vector3d(0.0, 1.0, 0.0);
        sphere->v = Vector3d(10.0, 15.0, 5.0);
        sphere->v0 = Vector3d(10.0, 15.0, 5.0);

        loadDataInputFile(DATA_DIR);

        // Create shapes
        for(const auto &mesh : meshData) {
            auto shape = make_shared<Shape>();
            shapes.push_back(shape);
            shape->loadMesh(DATA_DIR + mesh[0]);
            shape->setTextureFilename(mesh[1]);
            shape->init();
        }

        for(const auto &filename : textureData) {
            auto textureKd = make_shared<Texture>();
            textureMap[filename] = textureKd;
            textureKd->setFilename(DATA_DIR + filename);
            textureKd->setUnit(texUnit); // Bind to unit 1
            textureKd->init();
            textureKd->setWrapModes(GL_REPEAT, GL_REPEAT);
        }

	}

    sphereTexture = make_shared<Texture>();
    sphereTexture->setFilename(DATA_DIR + "white.png");
    sphereTexture->setUnit(texUnit); // Bind to unit 1
    sphereTexture->init();
    sphereTexture->setWrapModes(GL_REPEAT, GL_REPEAT);

}

void Scene::init()
{
    if (sphereShape)
        sphereShape->init();
}

void Scene::cleanup() {
    if (sphereShape)
        sphereShape->cleanupBuffers();
    for (auto &shape: shapes) {
        if (shape)
            shape->cleanupBuffers();
    }
    for (auto &[key, tex]: textureMap) {
        if (tex)
            tex->cleanupTexture();
    }
}

void Scene::setSceneNum(int sceneNum) {
    this->sceneIndex = sceneNum;
}

void Scene::tare()
{
    for(int i = 0; i < (int)spheres.size(); ++i) {
        spheres[i]->tare();
    }
}

void Scene::reset()
{
	t = 0.0;
    for (auto sphere: spheres) {
        sphere->x = sphere->x0;
        sphere->v = sphere->v0;
    }
}

void Scene::step()
{
    double cf = 1.0;
    if (!spheres.empty()) {
        // collision detection;
        for (auto s: spheres) {
            double f = s->detectCollision(h, shapes);
            cf = min(cf, f);
        }
        if (cf != 1.0) {
            std::cout<<"Collided "<<cf<<"\n";
        }

        for (auto s: spheres) {
            s->step(cf * h, forceFields, simParams);
        }
    }
    t += cf * h;

    // update wind
    Eigen::Vector3d wP(0.0, 0.0, -5.0);
    Eigen::Vector3d wDir(0.0, 0.0, -5.0);
    wDir.x() = 5.0 * sin(simParams.windOscilationSpeed * t);
    wind->setDirection(wDir);
    wind->setStrength(simParams.windStrength);

	// Simulate the ball
	// hair->step(h, forceFields, spheres);
}

void Scene::draw(std::shared_ptr<MatrixStack> MV, const shared_ptr<Program> prog) const
{
    for(const auto &shape : shapes) {
        textureMap.at(shape->getTextureFilename())->bind(prog->getUniform("kdTex"));
//        textureMap[shape->getTextureFilename()]->bind(prog->getUniform("kdTex"));
        glLineWidth(1.0f); // for wireframe
//        glUniformMatrix4fv(prog->getUniform("P"), 1, GL_FALSE, glm::value_ptr(P->topMatrix()));
//        glUniformMatrix4fv(prog->getUniform("MV"), 1, GL_FALSE, glm::value_ptr(MV->topMatrix()));
        glUniform3f(prog->getUniform("ka"), 0.1f, 0.1f, 0.1f);
        glUniform3f(prog->getUniform("ks"), 0.1f, 0.1f, 0.1f);
        glUniform1f(prog->getUniform("s"), 200.0f);
        shape->setProgram(prog);
        shape->draw();
        textureMap.at(shape->getTextureFilename())->unbind();
    }

    sphereTexture->bind(prog->getUniform("kdTex"));
    glUniform3f(prog->getUniform("ka"), 0.1f, 0.1f, 0.1f);
    glUniform3f(prog->getUniform("ks"), 0.1f, 0.1f, 0.1f);
    glUniform1f(prog->getUniform("s"), 200.0f);
    glUniform3fv(prog->getUniform("kdFront"), 1, Vector3f(1.0, 1.0, 1.0).data());
    for(int i = 0; i < (int)spheres.size(); ++i) {
        spheres[i]->draw(MV, prog);
    }
}

void Scene::updateSimParams(SimParams& simParams) {
    this->simParams = simParams;
}