#include <iostream>
#include <fstream>
#include <glm/gtc/type_ptr.hpp>
#include <tbb/tbb.h>

#include "Scene.h"
#include "Particle.h"
#include "Shape.h"
#include "Program.h"
#include "Texture.h"
#include "MatrixStack.h"
#include "Gravity.h"
#include "PointGravity.h"
#include "Wind.h"
#include "ChladniForce.h"

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
            ss >> value;
            mesh.push_back(value); // isObstacle
            meshData.push_back(mesh);
        } else if(key.compare("GENERATOR") == 0) {
            vector<string> generator;
            ss >> value;
            generator.push_back(value); // obj
            ss >> value;
            generator.push_back(value); // texture
            ss >> value;
            generator.push_back(value); // particle count
            ss >> value;
            generator.push_back(value); // start time
            ss >> value;
            generator.push_back(value); // end time
            ss >> value;
            generator.push_back(value); // lifetime
            generatorData.push_back(generator);
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
	forceFields.push_back(gravity);
    spheres.clear();
    meshData.clear();
    generatorData.clear();
    textureData.clear();
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
            bool isObstacle = (mesh[2] == "true");
            std::cout<<"isObstacle: "<<mesh[2]<<"\n";
            shape->setObstacle(isObstacle);
            shape->init();
        }

        for(const auto &generator : generatorData) {
            int particleCount = std::stoi(generator[2]);
            std::string meshName = generator[0];
            std::string texName = generator[1];
            double startTime = std::stod(generator[3]);
            double endTime = std::stod(generator[4]);
            double lifetime = std::stod(generator[5]);
            auto generatorShape = make_shared<Shape>(true, particleCount);
            generators.push_back(generatorShape);
            generatorShape->loadMesh(DATA_DIR + meshName);
            generatorShape->setTextureFilename(texName);
            generatorShape->setObstacle(false);
            generatorShape->init();
            std::vector<std::shared_ptr<Particle>> particles = generatorShape->generateParticles(sphereShape, simParams, 0.02,
                                                                                                 startTime, endTime, lifetime, h);
            spheres.insert(spheres.end(), particles.begin(), particles.end());
        }
        std::cout<<"generated\n";

        for(const auto &filename : textureData) {
            auto textureKd = make_shared<Texture>();
            textureMap[filename] = textureKd;
            textureKd->setFilename(DATA_DIR + filename);
            textureKd->setUnit(texUnit); // Bind to unit 1
            textureKd->init();
            textureKd->setWrapModes(GL_REPEAT, GL_REPEAT);
        }
	} else if (sceneIndex == 1) {
        sphereShape = make_shared<Shape>();
        sphereShape->loadMesh(RESOURCE_DIR + "sphere2.obj");

        loadDataInputFile(DATA_DIR, "multiSlopeInput.txt");

        // Create shapes
        for(const auto &mesh : meshData) {
            auto shape = make_shared<Shape>();
            shapes.push_back(shape);
            shape->loadMesh(DATA_DIR + mesh[0]);
            shape->setTextureFilename(mesh[1]);
            bool isObstacle = (mesh[2] == "true");
            std::cout<<"isObstacle: "<<mesh[2]<<"\n";
            shape->setObstacle(isObstacle);
            shape->init();
        }

        for(const auto &generator : generatorData) {
            int particleCount = std::stoi(generator[2]);
            std::string meshName = generator[0];
            std::string texName = generator[1];
            double startTime = std::stod(generator[3]);
            double endTime = std::stod(generator[4]);
            double lifetime = std::stod(generator[5]);
            auto generatorShape = make_shared<Shape>(true, particleCount);
            generators.push_back(generatorShape);
            generatorShape->loadMesh(DATA_DIR + meshName);
            generatorShape->setTextureFilename(texName);
            generatorShape->setObstacle(false);
            generatorShape->init();
            std::vector<std::shared_ptr<Particle>> particles = generatorShape->generateParticles(sphereShape, simParams, 0.02,
                                                                                                 startTime, endTime, lifetime, h);
            spheres.insert(spheres.end(), particles.begin(), particles.end());
        }
        std::cout<<"generated\n";

        for(const auto &filename : textureData) {
            auto textureKd = make_shared<Texture>();
            textureMap[filename] = textureKd;
            textureKd->setFilename(DATA_DIR + filename);
            textureKd->setUnit(texUnit); // Bind to unit 1
            textureKd->init();
            textureKd->setWrapModes(GL_REPEAT, GL_REPEAT);
        }
    } else if (sceneIndex == 2) {
        sphereShape = make_shared<Shape>();
        sphereShape->loadMesh(RESOURCE_DIR + "sphere2.obj");

        loadDataInputFile(DATA_DIR, "gravityPointInput.txt");

        // add point gravity force
        auto pointGravity = std::make_shared<PointGravity>(1e13, Eigen::Vector3d(0, 0, 0));
        auto pointGravity2 = std::make_shared<PointGravity>(1e13, Eigen::Vector3d(-2, -2, 0));
//        auto pointGravity3 = std::make_shared<PointGravity>(1e13, Eigen::Vector3d(-4, 2, 0));
//        auto pointGravity4 = std::make_shared<PointGravity>(1e13, Eigen::Vector3d(-6, -2, 0));
        forceFields.push_back(pointGravity);
        forceFields.push_back(pointGravity2);
//        forceFields.push_back(pointGravity3);
//        forceFields.push_back(pointGravity4);

        auto sphere1 = make_shared<Particle>(sphereShape, true);
        spheres.push_back(sphere1);
        sphere1->fixed = true;
        sphere1->r = 0.4;
        sphere1->x0 = Eigen::Vector3d(0, 0, 0);
        sphere1->x = sphere1->x0;
        auto sphere2 = make_shared<Particle>(sphereShape, true);
        spheres.push_back(sphere2);
        sphere2->fixed = true;
        sphere2->r = 0.4;
        sphere2->x0 = Eigen::Vector3d(-2, -2, 0);
        sphere2->x = sphere2->x0;

        // Create shapes
        for(const auto &mesh : meshData) {
            auto shape = make_shared<Shape>();
            shapes.push_back(shape);
            shape->loadMesh(DATA_DIR + mesh[0]);
            shape->setTextureFilename(mesh[1]);
            bool isObstacle = (mesh[2] == "true");
            std::cout<<"isObstacle: "<<mesh[2]<<"\n";
            shape->setObstacle(isObstacle);
            shape->init();
        }

        for(const auto &generator : generatorData) {
            int particleCount = std::stoi(generator[2]);
            std::string meshName = generator[0];
            std::string texName = generator[1];
            double startTime = std::stod(generator[3]);
            double endTime = std::stod(generator[4]);
            double lifetime = std::stod(generator[5]);
            auto generatorShape = make_shared<Shape>(true, particleCount);
            generators.push_back(generatorShape);
            generatorShape->loadMesh(DATA_DIR + meshName);
            generatorShape->setTextureFilename(texName);
            generatorShape->setObstacle(false);
            generatorShape->init();
            std::vector<std::shared_ptr<Particle>> particles = generatorShape->generateParticles(sphereShape, simParams, 0.1,
                                                                                                 startTime, endTime, lifetime, h);
            spheres.insert(spheres.end(), particles.begin(), particles.end());
        }
        std::cout<<"generated\n";

        for(const auto &filename : textureData) {
            auto textureKd = make_shared<Texture>();
            textureMap[filename] = textureKd;
            textureKd->setFilename(DATA_DIR + filename);
            textureKd->setUnit(texUnit); // Bind to unit 1
            textureKd->init();
            textureKd->setWrapModes(GL_REPEAT, GL_REPEAT);
        }
    } else if (sceneIndex == 3) {
        sphereShape = make_shared<Shape>();
        sphereShape->loadMesh(RESOURCE_DIR + "sphere2.obj");

        loadDataInputFile(DATA_DIR, "wedgeSlopeInput.txt");

        // Create shapes
        for(const auto &mesh : meshData) {
            auto shape = make_shared<Shape>();
            shapes.push_back(shape);
            shape->loadMesh(DATA_DIR + mesh[0]);
            shape->setTextureFilename(mesh[1]);
            bool isObstacle = (mesh[2] == "true");
            std::cout<<"isObstacle: "<<mesh[2]<<"\n";
            shape->setObstacle(isObstacle);
            shape->init();
        }

        for(const auto &generator : generatorData) {
            int particleCount = std::stoi(generator[2]);
            std::string meshName = generator[0];
            std::string texName = generator[1];
            double startTime = std::stod(generator[3]);
            double endTime = std::stod(generator[4]);
            double lifetime = std::stod(generator[5]);
            auto generatorShape = make_shared<Shape>(true, particleCount);
            generators.push_back(generatorShape);
            generatorShape->loadMesh(DATA_DIR + meshName);
            generatorShape->setTextureFilename(texName);
            generatorShape->setObstacle(false);
            generatorShape->init();
            std::vector<std::shared_ptr<Particle>> particles = generatorShape->generateParticles(sphereShape, simParams, 0.02,
                                                                                                 startTime, endTime, lifetime, h);
            spheres.insert(spheres.end(), particles.begin(), particles.end());
        }
        std::cout<<"generated\n";

        for(const auto &filename : textureData) {
            auto textureKd = make_shared<Texture>();
            textureMap[filename] = textureKd;
            textureKd->setFilename(DATA_DIR + filename);
            textureKd->setUnit(texUnit); // Bind to unit 1
            textureKd->init();
            textureKd->setWrapModes(GL_REPEAT, GL_REPEAT);
        }
    } else if (sceneIndex == 4) {
        sphereShape = make_shared<Shape>();
        sphereShape->loadMesh(RESOURCE_DIR + "sphere2.obj");

        loadDataInputFile(DATA_DIR, "sawInput.txt");

        // Load saw model
        sawShape = make_shared<Shape>();
        sawShape->setObstacle(false);
        sawShape->loadMesh(DATA_DIR + "saw.obj");
        sawTexture = make_shared<Texture>();
        sawTexture->setFilename(DATA_DIR + "grey.jpg");
        sawTexture->setUnit(texUnit); // Bind to unit 1
        sawTexture->init();
        sawTexture->setWrapModes(GL_REPEAT, GL_REPEAT);


        // Create shapes
        for(const auto &mesh : meshData) {
            auto shape = make_shared<Shape>();
            shapes.push_back(shape);
            shape->loadMesh(DATA_DIR + mesh[0]);
            shape->setTextureFilename(mesh[1]);
            bool isObstacle = (mesh[2] == "true");
            std::cout<<"isObstacle: "<<mesh[2]<<"\n";
            shape->setObstacle(isObstacle);
            shape->init();
        }

        for(const auto &generator : generatorData) {
            int particleCount = std::stoi(generator[2]);
            std::string meshName = generator[0];
            std::string texName = generator[1];
            double startTime = std::stod(generator[3]);
            double endTime = std::stod(generator[4]);
            double lifetime = std::stod(generator[5]);
            auto generatorShape = make_shared<Shape>(true, particleCount);
            generators.push_back(generatorShape);
            generatorShape->loadMesh(DATA_DIR + meshName);
            generatorShape->setTextureFilename(texName);
            generatorShape->setObstacle(false);
            generatorShape->init();
            std::vector<std::shared_ptr<Particle>> particles = generatorShape->generateParticles(sphereShape, simParams, 0.02,
                                                                                                 startTime, endTime, lifetime, h);
            spheres.insert(spheres.end(), particles.begin(), particles.end());
        }
        std::cout<<"generated\n";

        for(const auto &filename : textureData) {
            auto textureKd = make_shared<Texture>();
            textureMap[filename] = textureKd;
            textureKd->setFilename(DATA_DIR + filename);
            textureKd->setUnit(texUnit); // Bind to unit 1
            textureKd->init();
            textureKd->setWrapModes(GL_REPEAT, GL_REPEAT);
        }

        sawAngle = 0.0;
    } else if (sceneIndex == 5) {
        sphereShape = make_shared<Shape>();
        sphereShape->loadMesh(RESOURCE_DIR + "sphere2.obj");

        loadDataInputFile(DATA_DIR, "chladniInput.txt");

        // Create shapes
        for(const auto &mesh : meshData) {
            auto shape = make_shared<Shape>();
            shapes.push_back(shape);
            shape->loadMesh(DATA_DIR + mesh[0]);
            shape->setTextureFilename(mesh[1]);
            bool isObstacle = (mesh[2] == "true");
            std::cout<<"isObstacle: "<<mesh[2]<<"\n";
            shape->setObstacle(isObstacle);
            shape->init();
        }

        for(const auto &generator : generatorData) {
            int particleCount = std::stoi(generator[2]);
            std::string meshName = generator[0];
            std::string texName = generator[1];
            double startTime = std::stod(generator[3]);
            double endTime = std::stod(generator[4]);
            double lifetime = std::stod(generator[5]);
            auto generatorShape = make_shared<Shape>(true, particleCount);
            generators.push_back(generatorShape);
            generatorShape->loadMesh(DATA_DIR + meshName);
            generatorShape->setTextureFilename(texName);
            generatorShape->setObstacle(false);
            generatorShape->init();
            std::vector<std::shared_ptr<Particle>> particles = generatorShape->generateParticles(sphereShape, simParams, 0.02,
                                                                                                 startTime, endTime, lifetime, h);
            spheres.insert(spheres.end(), particles.begin(), particles.end());
        }
        std::cout<<"generated\n";

        for(const auto &filename : textureData) {
            auto textureKd = make_shared<Texture>();
            textureMap[filename] = textureKd;
            textureKd->setFilename(DATA_DIR + filename);
            textureKd->setUnit(texUnit); // Bind to unit 1
            textureKd->init();
            textureKd->setWrapModes(GL_REPEAT, GL_REPEAT);
        }

        auto chladniForce = std::make_shared<ChladniForce>();
        chladniForce->setA(simParams.chladniA);
        chladniForce->setB(simParams.chladniB);
        chladniForce->setM(simParams.chladniM);
        chladniForce->setN(simParams.chladniN);

        forceFields.push_back(chladniForce);
    }

    sphereTexture = make_shared<Texture>();
    if (sceneIndex == 4) {
        sphereTexture->setFilename(DATA_DIR + "yellow.jpeg");
    } else {
        sphereTexture->setFilename(DATA_DIR + "white.png");
    }
    sphereTexture->setUnit(texUnit); // Bind to unit 1
    sphereTexture->init();
    sphereTexture->setWrapModes(GL_REPEAT, GL_REPEAT);

}

void Scene::init()
{
    // Units: meters, kilograms, seconds
    h = simParams.timestep;
    std::cout<<"timestep set: "<<simParams.timestep<<"\n";
    std::cout<<"timestep used: "<<h<<"\n";
    if (sphereShape)
        sphereShape->init();
    if (sawShape)
        sawShape->init();
}

void Scene::cleanup() {
    if (sphereShape)
        sphereShape->cleanupBuffers();
    if (sawShape)
        sawShape->cleanupBuffers();
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

void Scene::step(std::ofstream &outputFile, bool writeToFile)
{
//    std::cout<<"timestep: "<<h<<"\n";
    if (!spheres.empty()) {
        // collision detection and step
        tbb::parallel_for((size_t)0, spheres.size(), [=](size_t i) {
            auto s = spheres[i];
            if (t*1e3 >= s->tStart && t*1e3 <= s->tEnd) {
                s->detectCollision(h, shapes);
                s->step(h, forceFields, simParams);
            }
        });
//        for (auto s: spheres) {
//            if (t*1e3 >= s->tStart && t*1e3 <= s->tEnd) {
//                s->detectCollision(h, shapes);
//                s->step(h, forceFields, simParams);
//            }
//        }
    }
    if (sceneIndex == 4) {
        sawAngle -= (h * 4);
        if (sawAngle <= -360) sawAngle = 0.0;
    }
    if (writeToFile) {
        if (!spheres.empty()) {
            int deadSpheres = 0;
            for (auto s: spheres) {
                bool alive = t*1e3 >= s->tStart && t*1e3 <= s->tEnd;
                if (!alive || s->fixed) deadSpheres++;
            }
            if (deadSpheres != spheres.size()) {
                outputFile << "timestep\n";
                for (auto s: spheres) {
                    bool alive = t*1e3 >= s->tStart && t*1e3 <= s->tEnd;
                    outputFile << (alive ? "a" : "d") << " " << s->x.x() << " " << s->x.y() << " " << s->x.z() << " \n";
                }
                outputFile <<"\n";
            }
        }
    }
    t += h;
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

    for(const auto &shape : generators) {
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

    // draw saw
    if (sceneIndex == 4) {
        MV->pushMatrix();
        MV->rotate(sawAngle, glm::vec3(1.0, 0.0, 0.0));
        sawTexture->bind(prog->getUniform("kdTex"));
        glUniform3f(prog->getUniform("ka"), 0.1f, 0.1f, 0.1f);
        glUniform3f(prog->getUniform("ks"), 0.1f, 0.1f, 0.1f);
        glUniform1f(prog->getUniform("s"), 200.0f);
        glUniform3fv(prog->getUniform("kdFront"), 1, Vector3f(1.0, 1.0, 1.0).data());
        glUniformMatrix4fv(prog->getUniform("MV"), 1, GL_FALSE, glm::value_ptr(MV->topMatrix()));
        sawShape->setProgram(prog);
        sawShape->draw();
        sawTexture->unbind();
        MV->popMatrix();
    }

    sphereTexture->bind(prog->getUniform("kdTex"));
    glUniform3f(prog->getUniform("ka"), 0.1f, 0.1f, 0.1f);
    glUniform3f(prog->getUniform("ks"), 0.1f, 0.1f, 0.1f);
    glUniform1f(prog->getUniform("s"), 200.0f);
    glUniform3fv(prog->getUniform("kdFront"), 1, Vector3f(1.0, 1.0, 1.0).data());
//    std::cout<<"t: "<<t<<"\n";
    for(int i = 0; i < (int)spheres.size(); ++i) {
//        spheres[i]->draw(MV, prog);
        if (spheres[i]->fixed || (t*1e3 >= spheres[i]->tStart && t*1e3 <= spheres[i]->tEnd)) {
            spheres[i]->draw(MV, prog);
        }
    }
    sphereTexture->unbind();

}

void Scene::updateSimParams(SimParams& simParams) {
    this->simParams = simParams;
}