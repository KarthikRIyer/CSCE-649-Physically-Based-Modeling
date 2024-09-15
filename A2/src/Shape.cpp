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

std::vector<std::shared_ptr<Particle>> Shape::generateParticles(const std::shared_ptr<Shape>& s, SimParams& simParams) const {
    std::vector<std::shared_ptr<Particle>> particles;
    if (isGenerator) {
        // https://stackoverflow.com/questions/9878965/rand-between-0-and-1
        std::mt19937_64 rng;
        uint64_t randomSeed = 123456;
        std::seed_seq ss{uint32_t(randomSeed & 0xffffffff), uint32_t(randomSeed >> 32)};
        rng.seed(ss);
        std::uniform_real_distribution<double> unif(0, 1);

        int totalParticles = particleCount;
        for (int i = 0; i < particleFractions.size(); i++) {
            int particlesToGenerate = (int) ((double) particleCount * particleFractions[i]);
            totalParticles -= particlesToGenerate;
            if (i == particleFractions.size()) { // if any particles are pending at the end assign them to last poly
                particlesToGenerate += totalParticles;
                totalParticles = 0;
            }
            auto polygon = polygons[i];
            Eigen::Vector3d P = polygon.points[0];
            Eigen::Vector3d Q = polygon.points[1];
            Eigen::Vector3d R = polygon.points[2];
            Eigen::Vector3d PQ = Q - P;
            Eigen::Vector3d PR = R - P;
            Eigen::Vector3d n = PQ.cross(PR);
            n.normalize();
            for (int j = 0; j < particlesToGenerate; j++) { // generate random point in triangle
//                https://blogs.sas.com/content/iml/2020/10/19/random-points-in-triangle.html
                double u = unif(rng);
                double v = unif(rng);
                if (u + v > 1.0) {
                    u = 1.0 - u;
                    v = 1.0 - v;
                }

                Eigen::Vector3d point = P + u * PQ + v * PR;

                auto sphere = make_shared<Particle>(s, true);
                particles.push_back(sphere);
                sphere->r = 0.01;
                sphere->x0 = point;
                sphere->x = sphere->x0;
                sphere->v0 = n * simParams.initialParticleVelocity;
                sphere->v = sphere->v0;
            }
        }
    }
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
