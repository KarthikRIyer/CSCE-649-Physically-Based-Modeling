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
#include "Grains.h"

using namespace std;
using namespace Eigen;

void Scene::loadDataInputFile(const std::string &DATA_DIR, const std::string &FILE_NAME)
{
    string filename = DATA_DIR + FILE_NAME;
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
        } else if(key.compare("GRAINS") == 0) {
            value.clear();
            ss >> value; // point file
            grainsData.push_back(value);
        } else if(key.compare("STATICGRAINS") == 0) {
            value.clear();
            ss >> value; // point file
            staticGrainsData.push_back(value);
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

	gravity = std::make_shared<Gravity>(Eigen::Vector3d(0.0, -9.8, 0.0));
	wind = std::make_shared<Wind>(0.0, Eigen::Vector3d(1.0, 0.0, 0.0));
	forceFields.push_back(gravity);
	forceFields.push_back(wind);

	if (sceneIndex == 0) {
        sphereShape = make_shared<Shape>();
        sphereShape->loadMesh(RESOURCE_DIR + "sphere2.obj");

        loadDataInputFile(DATA_DIR, "input.txt");

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
        grains = std::make_shared<Grains>(30*30*30, 0.02, 1, sphereShape);

	} else if (sceneIndex == 1) {
        sphereShape = make_shared<Shape>();
        sphereShape->loadMesh(RESOURCE_DIR + "sphere2.obj");

        loadDataInputFile(DATA_DIR, "inputFunnel.txt");

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
        grains = std::make_shared<Grains>(30*30*30, 0.02, 1, sphereShape);
    } else if (sceneIndex == 2) {
        sphereShape = make_shared<Shape>();
        sphereShape->loadMesh(RESOURCE_DIR + "sphere2.obj");

        loadDataInputFile(DATA_DIR, "inputText.txt");

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

        std::vector<Eigen::Vector3d> points;
        for(const auto &filename : grainsData) {
            std::string finalPath = DATA_DIR + filename;
            std::vector<Eigen::Vector3d> p = readGrainsFile(finalPath);
            points.insert(points.end(), p.begin(), p.end());
        }
        double floorPos = -0.8;
        std::vector<Eigen::Vector3d> staticPoints;
        for(const auto &filename : staticGrainsData) {
            std::string finalPath = DATA_DIR + filename;
            std::vector<Eigen::Vector3d> p = readGrainsFile(finalPath);
            for (Eigen::Vector3d& v: p) v += Eigen::Vector3d(0, floorPos, 0);
            staticPoints.insert(staticPoints.end(), p.begin(), p.end());
        }

        grains = std::make_shared<Grains>(points, staticPoints, 0.02, 1, sphereShape);
    } else if (sceneIndex == 3) {
        sphereShape = make_shared<Shape>();
        sphereShape->loadMesh(RESOURCE_DIR + "sphere2.obj");

        loadDataInputFile(DATA_DIR, "inputSandtower.txt");

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

        std::vector<Eigen::Vector3d> points;
        for(const auto &filename : grainsData) {
            std::string finalPath = DATA_DIR + filename;
            std::vector<Eigen::Vector3d> p = readGrainsFile(finalPath);
            points.insert(points.end(), p.begin(), p.end());
        }

        double floorPos = -0.8;
        std::vector<Eigen::Vector3d> staticPoints;
        for(const auto &filename : staticGrainsData) {
            std::string finalPath = DATA_DIR + filename;
            std::vector<Eigen::Vector3d> p = readGrainsFile(finalPath);
            for (Eigen::Vector3d& v: p) v += Eigen::Vector3d(0, floorPos, 0);
            staticPoints.insert(staticPoints.end(), p.begin(), p.end());
        }

        grains = std::make_shared<Grains>(points, staticPoints, 0.01, 1, sphereShape);
    } else if (sceneIndex == 4) {
        sphereShape = make_shared<Shape>();
        sphereShape->loadMesh(RESOURCE_DIR + "sphere2.obj");

        loadDataInputFile(DATA_DIR, "inputMonkey.txt");

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

        std::vector<Eigen::Vector3d> points;
        for(const auto &filename : grainsData) {
            std::string finalPath = DATA_DIR + filename;
            std::vector<Eigen::Vector3d> p = readGrainsFile(finalPath);
            points.insert(points.end(), p.begin(), p.end());
        }

        double floorPos = -0.8;
        std::vector<Eigen::Vector3d> staticPoints;
        for(const auto &filename : staticGrainsData) {
            std::string finalPath = DATA_DIR + filename;
            std::vector<Eigen::Vector3d> p = readGrainsFile(finalPath);
            for (Eigen::Vector3d& v: p) v += Eigen::Vector3d(0, floorPos, 0);
            staticPoints.insert(staticPoints.end(), p.begin(), p.end());
        }

        grains = std::make_shared<Grains>(points, staticPoints, 0.01, 1, sphereShape);
    } else if (sceneIndex == 5) {
        sphereShape = make_shared<Shape>();
        sphereShape->loadMesh(RESOURCE_DIR + "sphere2.obj");

        loadDataInputFile(DATA_DIR, "inputCSCEText.txt");

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

        std::vector<Eigen::Vector3d> points;
        for(const auto &filename : grainsData) {
            std::string finalPath = DATA_DIR + filename;
            std::vector<Eigen::Vector3d> p = readGrainsFile(finalPath);
            points.insert(points.end(), p.begin(), p.end());
        }

        double floorPos = -0.8;
        std::vector<Eigen::Vector3d> staticPoints;
        for(const auto &filename : staticGrainsData) {
            std::string finalPath = DATA_DIR + filename;
            std::vector<Eigen::Vector3d> p = readGrainsFile(finalPath);
            for (Eigen::Vector3d& v: p) v += Eigen::Vector3d(0, floorPos, 0);
            staticPoints.insert(staticPoints.end(), p.begin(), p.end());
        }

        grains = std::make_shared<Grains>(points, staticPoints, 0.01, 1, sphereShape);
    }


    sphereTexture = std::make_shared<Texture>();
    sphereTexture->setFilename(DATA_DIR + "white.png");
    sphereTexture->setUnit(texUnit); // Bind to unit 1
    sphereTexture->init();
    sphereTexture->setWrapModes(GL_REPEAT, GL_REPEAT);
}

std::vector<Eigen::Vector3d> Scene::readGrainsFile(const std::string& filename) {
    ifstream in;
    in.open(filename);
    if(!in.good()) {
        cout << "Cannot read " << filename << endl;
        return std::vector<Eigen::Vector3d>();
    }
    cout << "Loading " << filename << endl;
    std::vector<Eigen::Vector3d> points;
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
        string xstr, ystr, zstr;
        stringstream ss(line);
        // key
        ss >> xstr;
        ss >> ystr;
        ss >> zstr;
        double xVal = std::stod(xstr);
        double yVal = std::stod(ystr);
        double zVal = std::stod(zstr);
        Eigen::Vector3d v(xVal, yVal, zVal);
        points.push_back(v);
    }
    in.close();
    return points;
}

void Scene::init()
{
    // Units: meters, kilograms, seconds
    h = simParams.timestep;
    std::cout<<"timestep set: "<<simParams.timestep<<"\n";
    std::cout<<"timestep used: "<<h<<"\n";
    if (sphereShape)
        sphereShape->init();
    if (grains)
        grains->reset();
//    if (wind)
//        wind->setStrength(simParams.windStrength);
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

    if (grains)
        grains->reset();
}

void Scene::step(std::ofstream &outputFile, bool writeToFile)
{
//    std::cout<<"timestep: "<<h<<"\n";
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

    if (grains) {
        grains->step(h, forceFields, shapes, simParams);
        if (writeToFile) {
            outputFile << "timestep\n";
            for (const std::shared_ptr<Particle>& particle: grains->getParticles()) {
                if (particle->fixed) continue;
                outputFile << particle->x.x() << " " << particle->x.y() << " " << particle->x.z() << " \n";
            }
        }
    }

    t += cf * h;

    // update wind
//    Eigen::Vector3d wP(0.0, 0.0, 0.0);
//    Eigen::Vector3d wDir(0.0, 0.0, 0.0);
//    wDir.x() = 5.0 * sin(simParams.windOscilationSpeed * t);
//    wind->setDirection(wDir);
//    wind->setStrength(simParams.windStrength);
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
    if (grains)
        grains->draw(MV, prog);
}

void Scene::updateSimParams(SimParams& simParams) {
    this->simParams = simParams;
}