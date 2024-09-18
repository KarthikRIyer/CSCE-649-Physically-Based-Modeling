#include <iostream>
#include <fstream>

#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"

#include <random>

#include "Shape.h"
#include "GLSL.h"
#include "Program.h"
#include "Polygon.h"
#include "Particle.h"
#include "SimParams.h"

using namespace std;
using namespace glm;

struct Vertex {
    glm::vec3 Position;
    glm::vec3 Normal;
    glm::vec2 TexCoords;
};

Shape::Shape() :
        prog(NULL),
        posBufID(0),
        norBufID(0),
        texBufID(0),
        isGenerator(false),
        particleCount(0)
{
}

Shape::Shape(bool isGenerator, int particleCount) :
        prog(NULL),
        posBufID(0),
        norBufID(0),
        texBufID(0),
        isGenerator(isGenerator),
        particleCount(particleCount)
{
}

Shape::~Shape()
{
}

void Shape::loadObj(const string &filename, vector<float> &pos, vector<float> &nor, vector<float> &tex, bool loadNor, bool loadTex)
{

    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;
    std::string warn;
    std::string err;
    bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err, filename.c_str());
    if(!warn.empty()) {
        //std::cout << warn << std::endl;
    }
    if(!err.empty()) {
        std::cerr << err << std::endl;
    }
    if(!ret) {
        return;
    }
    // Loop over shapes
    for (size_t s = 0; s < shapes.size(); s++) {
        // Loop over faces(polygon)
        size_t index_offset = 0;
        for(size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++) {
            int fv = shapes[s].mesh.num_face_vertices[f];
            // Loop over vertices in the face.
            Polygon polygon;
            for(size_t v = 0; v < fv; v++) {
                // access to vertex
                tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];
                pos.push_back(attrib.vertices[3*idx.vertex_index+0]);
                pos.push_back(attrib.vertices[3*idx.vertex_index+1]);
                pos.push_back(attrib.vertices[3*idx.vertex_index+2]);

                polygon.addPoint(attrib.vertices[3*idx.vertex_index+0],
                                 attrib.vertices[3*idx.vertex_index+1],
                                 attrib.vertices[3*idx.vertex_index+2]);

                if(!attrib.normals.empty() && loadNor) {
                    nor.push_back(attrib.normals[3*idx.normal_index+0]);
                    nor.push_back(attrib.normals[3*idx.normal_index+1]);
                    nor.push_back(attrib.normals[3*idx.normal_index+2]);
                }
                if(!attrib.texcoords.empty() && loadTex) {
                    tex.push_back(attrib.texcoords[2*idx.texcoord_index+0]);
                    tex.push_back(attrib.texcoords[2*idx.texcoord_index+1]);
                }
            }
            polygons.push_back(polygon);
            index_offset += fv;
        }
    }
}

std::vector<Polygon>& Shape::getPolygons() {
    return polygons;
}

void Shape::loadMesh(const string &meshName)
{
    // Load geometry
    meshFilename = meshName;
    loadObj(meshFilename, posBuf, norBuf, texBuf);
    std::cout<<"Loaded obj\n";

    if (isGenerator) {
        particleFractions.clear();
        double totalArea = 0.0;
        for (auto polygon: polygons) {
            Eigen::Vector3d P = polygon.points[0];
            Eigen::Vector3d Q = polygon.points[1];
            Eigen::Vector3d R = polygon.points[2];
            Eigen::Vector3d u = P - Q;
            Eigen::Vector3d v = P - R;

            double area = 0.5 * u.cross(v).norm();
            totalArea += area;
            particleFractions.push_back(area);
        }
        for (double & particleFraction : particleFractions) {
            particleFraction /= totalArea;
        }
        std::cout<<"Calculated particle fractions per tri\n";
    }
}

std::vector<std::shared_ptr<Particle>> Shape::generateParticles(const std::shared_ptr<Shape>& s, SimParams& simParams, double particleSize,
                                                                double startTime, double endTime, double lifetime, double h) const {
    std::vector<std::shared_ptr<Particle>> particles;
    if (isGenerator) {
        // https://stackoverflow.com/questions/9878965/rand-between-0-and-1
        std::mt19937_64 rng;
        uint64_t randomSeed = 123456;
        std::seed_seq ss{uint32_t(randomSeed & 0xffffffff), uint32_t(randomSeed >> 32)};
        rng.seed(ss);
        std::uniform_real_distribution<double> unif(0, 1);

        std::mt19937_64 rngVel;
        uint64_t randomSeedVel = 1234568765;
        std::seed_seq ssVel{uint32_t(randomSeedVel & 0xffffffff), uint32_t(randomSeed >> 32)};
        rngVel.seed(ssVel);
        std::uniform_real_distribution<double> unifVelTheta(0, simParams.initialParticleVelocityRandomness * (M_PI / 180.0));
        std::uniform_real_distribution<double> unifVelPhi(0, 2.0 * M_PI);

        int totalTimesteps = (endTime - startTime)/(h * 1e3);
        int totalParticles = particleCount;
        std::cout<<"totalTimesteps: "<<totalTimesteps<<"\n";
        std::cout<<"totalParticles: "<<totalParticles<<"\n";
        for (int i = 0; i < particleFractions.size(); i++) {
            int particlesToGenerate = (int) ((double) particleCount * particleFractions[i]);
            totalParticles -= particlesToGenerate;
            if (i == particleFractions.size()) { // if any particles are pending at the end assign them to last poly
                particlesToGenerate += std::max(totalParticles, 0);
                totalParticles = 0;
            }
            int particlesPerTimestep = particlesToGenerate / totalTimesteps;
            int everyXTimestep = 0;
            if (particlesPerTimestep == 0) {
                double fractionalParticlesPerTimestep = (double) particlesToGenerate / (double) totalTimesteps;
                double timestepsToSkip = 1.0 / fractionalParticlesPerTimestep;
                everyXTimestep = timestepsToSkip;
                particlesPerTimestep = 1;
            }
            std::cout<<"particlesToGenerate: "<<particlesToGenerate<<"\n";
            std::cout<<"particlesPerTimestep: "<<particlesPerTimestep<<"\n";
            auto polygon = polygons[i];
            Eigen::Vector3d P = polygon.points[0];
            Eigen::Vector3d Q = polygon.points[1];
            Eigen::Vector3d R = polygon.points[2];
            Eigen::Vector3d PQ = Q - P;
            Eigen::Vector3d PR = R - P;
            Eigen::Vector3d n = PQ.cross(PR).normalized();
            Eigen::Vector3d t = PQ.normalized();
            Eigen::Vector3d bt = n.cross(t).normalized();
            Eigen::Matrix3d Rmat;
            Rmat.col(0) = t;
            Rmat.col(1) = bt;
            Rmat.col(2) = n;

            int particlesGenerated = 0;
            for (int j = 0; j < totalTimesteps; j++) {
                if (everyXTimestep != 0 && j % everyXTimestep != 0) continue;
                for (int k = 0; k < particlesPerTimestep; k++) {
                    if (particlesGenerated > particlesToGenerate)
                        continue;
                    particlesGenerated++;
                    double startTime = j * h * 1e3;
                    double endTime = startTime + lifetime;

                    // generate random point in triangle
                    // https://blogs.sas.com/content/iml/2020/10/19/random-points-in-triangle.html
                    double u = unif(rng);
                    double v = unif(rng);
                    if (u + v > 1.0) {
                        u = 1.0 - u;
                        v = 1.0 - v;
                    }

                    Eigen::Vector3d point = P + u * PQ + v * PR;

                    auto sphere = make_shared<Particle>(s, true);
                    particles.push_back(sphere);
                    sphere->fixed = false;
                    sphere->r = particleSize;
                    sphere->x0 = point;
                    sphere->x = sphere->x0;
                    sphere->tStart = startTime;
                    sphere->tEnd = endTime;
                    sphere->v0 = n * simParams.initialParticleVelocity;

                    // add randomness to velocity direction
                    double theta = unifVelTheta(rngVel);
                    double phi = unifVelPhi(rngVel);
                    Eigen::Vector3d y(std::sin(theta) * std::cos(phi), std::sin(theta) * std::sin(phi), std::cos(theta));
                    sphere->v0 = Rmat * y * simParams.initialParticleVelocity;;

                    sphere->v = sphere->v0;
                }
            }
        }
    }
    std::cout<<"Particles size: "<<particles.size()<<"\n";
    return particles;
}

void Shape::init()
{
    glGenVertexArrays(1, &VAO);
    glBindVertexArray(VAO);


    glGenBuffers(1, &posBufID);
    glGenBuffers(1, &norBufID);
    glGenBuffers(1, &texBufID);

    // Send the position array to the GPU
    glBindBuffer(GL_ARRAY_BUFFER, posBufID);
    glBufferData(GL_ARRAY_BUFFER, posBuf.size()*sizeof(float), &posBuf[0], GL_STATIC_DRAW);

    // Send the normal array to the GPU
    glBindBuffer(GL_ARRAY_BUFFER, norBufID);
    glBufferData(GL_ARRAY_BUFFER, norBuf.size()*sizeof(float), &norBuf[0], GL_STATIC_DRAW);

    // Send the texcoord array to the GPU

    glBindBuffer(GL_ARRAY_BUFFER, texBufID);
    glBufferData(GL_ARRAY_BUFFER, texBuf.size()*sizeof(float), &texBuf[0], GL_STATIC_DRAW);

    // Unbind the arrays
//    glBindBuffer(GL_ARRAY_BUFFER, 0);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*) offsetof(Vertex, Normal));
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*) offsetof(Vertex, TexCoords));
    glBindVertexArray(0);
    GLSL::checkError(GET_FILE_LINE);
}

void Shape::cleanupBuffers() {
    glDeleteBuffers(1, &posBufID);
    glDeleteBuffers(1, &norBufID);
    glDeleteBuffers(1, &texBufID);
    glDeleteVertexArrays(1, &VAO);
}

void Shape::draw() const
{
    assert(prog);
    glBindVertexArray(VAO);
    int h_pos = prog->getAttribute("aPos");
    glEnableVertexAttribArray(h_pos);
    glBindBuffer(GL_ARRAY_BUFFER, posBufID);
    glVertexAttribPointer(h_pos, 3, GL_FLOAT, GL_FALSE, 0, (const void *)0);

    int h_nor = prog->getAttribute("aNor");
    glEnableVertexAttribArray(h_nor);
    glBindBuffer(GL_ARRAY_BUFFER, norBufID);
    glVertexAttribPointer(h_nor, 3, GL_FLOAT, GL_FALSE, 0, (const void *)0);

    int h_tex = prog->getAttribute("aTex");
    glEnableVertexAttribArray(h_tex);
    glBindBuffer(GL_ARRAY_BUFFER, texBufID);
    glVertexAttribPointer(h_tex, 2, GL_FLOAT, GL_FALSE, 0, (const void *)0);

    // Draw
    int count = (int)posBuf.size()/3; // number of indices to be rendered
    glDrawArrays(GL_TRIANGLES, 0, count);

    glDisableVertexAttribArray(h_tex);
    glDisableVertexAttribArray(h_nor);
    glDisableVertexAttribArray(h_pos);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
    GLSL::checkError(GET_FILE_LINE);
}
