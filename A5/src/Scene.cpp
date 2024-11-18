#include <iostream>
#include <fstream>
#include <glm/gtc/type_ptr.hpp>
#include <tbb/tbb.h>

#include "Scene.h"
#include "Shape.h"
#include "RigidBody.h"
#include "Program.h"
#include "Texture.h"
#include "MatrixStack.h"
#include "Gravity.h"
#include "Particle.h"
#include "Wind.h"

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
        } else if(key.compare("RB") == 0) {
            vector<string> mesh;
            ss >> value;
            mesh.push_back(value); // obj
            ss >> value;
            mesh.push_back(value); // texture
            ss >> value;

            mesh.push_back(value); // pos x
            ss >> value;
            mesh.push_back(value); // pos y
            ss >> value;
            mesh.push_back(value); // pos z
            ss >> value;

            mesh.push_back(value); // v x
            ss >> value;
            mesh.push_back(value); // v y
            ss >> value;
            mesh.push_back(value); // v z
            ss >> value;

            mesh.push_back(value); // angV x
            ss >> value;
            mesh.push_back(value); // angV y
            ss >> value;
            mesh.push_back(value); // angV z
            ss >> value;

            mesh.push_back(value); // rot x
            ss >> value;
            mesh.push_back(value); // rot y
            ss >> value;
            mesh.push_back(value); // rot z
            ss >> value;
            rigidBodyData.push_back(mesh);
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

        for(const auto &mesh : rigidBodyData) {
            Eigen::Vector3d pos(std::stod(mesh[2]), std::stod(mesh[3]), std::stod(mesh[4]) );
            Eigen::Vector3d v(std::stod(mesh[5]), std::stod(mesh[6]), std::stod(mesh[7]) );
            Eigen::Vector3d angV(std::stod(mesh[8]), std::stod(mesh[9]), std::stod(mesh[10]) );

            double xRot = std::stod(mesh[11]);
            double yRot = std::stod(mesh[12]);
            double zRot = std::stod(mesh[13]);
            xRot *= (M_PI / 180.0);
            yRot *= (M_PI / 180.0);
            zRot *= (M_PI / 180.0);

            Eigen::AngleAxisd rollAngle(yRot, Eigen::Vector3d::UnitY());
            Eigen::AngleAxisd yawAngle(zRot, Eigen::Vector3d::UnitZ());
            Eigen::AngleAxisd pitchAngle(xRot, Eigen::Vector3d::UnitX());
            Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;

            Eigen::Matrix3d rotMat = q.matrix();

//            std::cout<<"v init: "<<v.transpose()<<"\n";
            auto rigidBody = std::make_shared<RigidBody>(1.0, pos,
                                                    v, angV, rotMat);
            rigidBodies.push_back(rigidBody);
            rigidBody->loadMesh(DATA_DIR + mesh[0]);
            rigidBody->setTextureFilename(mesh[1]);
            rigidBody->setObstacle(true);
            rigidBody->init();
        }


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

        loadDataInputFile(DATA_DIR, "inputbox.txt");

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

        for(const auto &mesh : rigidBodyData) {
            Eigen::Vector3d pos(std::stod(mesh[2]), std::stod(mesh[3]), std::stod(mesh[4]) );
            Eigen::Vector3d v(std::stod(mesh[5]), std::stod(mesh[6]), std::stod(mesh[7]) );
            Eigen::Vector3d angV(std::stod(mesh[8]), std::stod(mesh[9]), std::stod(mesh[10]) );
//            std::cout<<"v init: "<<v.transpose()<<"\n";
            auto rigidBody = std::make_shared<RigidBody>(1.0, pos,
                                                         v, angV);
            rigidBodies.push_back(rigidBody);
            rigidBody->loadMesh(DATA_DIR + mesh[0]);
            rigidBody->setTextureFilename(mesh[1]);
            rigidBody->setObstacle(true);
            rigidBody->init();
        }


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

        loadDataInputFile(DATA_DIR, "inputWedge.txt");

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

        for(const auto &mesh : rigidBodyData) {
            Eigen::Vector3d pos(std::stod(mesh[2]), std::stod(mesh[3]), std::stod(mesh[4]) );
            Eigen::Vector3d v(std::stod(mesh[5]), std::stod(mesh[6]), std::stod(mesh[7]) );
            Eigen::Vector3d angV(std::stod(mesh[8]), std::stod(mesh[9]), std::stod(mesh[10]) );
//            std::cout<<"v init: "<<v.transpose()<<"\n";
            auto rigidBody = std::make_shared<RigidBody>(1.0, pos,
                                                         v, angV);
            rigidBodies.push_back(rigidBody);
            rigidBody->loadMesh(DATA_DIR + mesh[0]);
            rigidBody->setTextureFilename(mesh[1]);
            rigidBody->setObstacle(true);
            rigidBody->init();
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

    for (int i = 0; i < 100; ++i) {
        Particle p(sphereShape, true);
        p.x = Eigen::Vector3d(0, 0, 0);
        particles.push_back(p);
    }
}

void Scene::init()
{
    // Units: meters, kilograms, seconds
    h = simParams.timestep;
    std::cout<<"timestep set: "<<simParams.timestep<<"\n";
    std::cout<<"timestep used: "<<h<<"\n";
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

    for (const auto & rigidBody : rigidBodies) {
        rigidBody->reset();
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

    cX.clear();
    for (int i = 0; i < rigidBodies.size(); ++i) {
        rigidBodies[i]->step(h, forceFields, simParams);
        rigidBodies[i]->detectCollision(h, shapes, simParams, cX);
        rigidBodies[i]->detectEdgeCollision(h, shapes, simParams);
    }
    for (int i = 0; i < cX.size(); ++i) {
        particles[i].x = cX[i];
    }

    /*
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
    */
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

//    for (int i = 0; i < cX.size(); ++i) {
//        particles[i].draw(MV, prog);
//    }

    for(const auto &rigidBody : rigidBodies) {
        textureMap.at(rigidBody->getTextureFilename())->bind(prog->getUniform("kdTex"));
        glLineWidth(1.0f); // for wireframe
//        glUniform3f(prog->getUniform("ka"), 0.1f, 0.1f, 0.1f);
//        glUniform3f(prog->getUniform("ks"), 0.1f, 0.1f, 0.1f);
//        glUniform1f(prog->getUniform("s"), 200.0f);
//        rigidBody->setProgram(prog);
        rigidBody->draw(MV, prog);
        textureMap.at(rigidBody->getTextureFilename())->unbind();
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
//    std::cout<<"Using integration method: " << simParams.integrationMethod<<"\n";
    this->simParams = simParams;
}