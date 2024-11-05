//
// Created by Karthik Iyer on 04/11/24.
//

#include "RigidBody.h"

#include <iostream>
#include <fstream>

#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "tiny_obj_loader.h"

#include <random>

#include "MatrixStack.h"
#include "RigidBody.h"
#include "GLSL.h"
#include "Program.h"
#include "Polygon.h"
#include "SimParams.h"
#include "Vertex.h"
#include "IForceField.h"

using namespace std;
using namespace glm;

RigidBody::RigidBody(double m, Eigen::Vector3d pos, Eigen::Vector3d v, Eigen::Vector3d angV) :
        mass(m),
        x0(pos),
        x(pos),
        v0(v),
        v(v),
        angV0(angV),
        angV(angV),
        prog(NULL),
        posBufID(0),
        norBufID(0),
        texBufID(0),
        isObstacle(true)
{
    R0 = Eigen::Matrix3d::Identity();
    R = R0;
}

RigidBody::~RigidBody()
{
}

void RigidBody::loadObj(const string &filename, vector<float> &pos, vector<float> &nor, vector<float> &tex, bool loadNor, bool loadTex)
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
            polygons0.push_back(polygon);
            index_offset += fv;
        }
    }
    computeMomentOfInertia();
}

void RigidBody::computeMomentOfInertia() {

}

void RigidBody::reset() {
    for (int i = 0; i < polygons.size(); ++i) {
        for (int j = 0; j < polygons[i].points.size(); ++j) {
            polygons[i].points[j] =  polygons0[i].points[j];
        }
    }
    x = x0;
    v = v0;
    angV = angV0;
    R = R0;
}

inline glm::mat4 RigidBody::convertToGLMMat(Eigen::Matrix3d rot) {
    glm::mat4 glmR = glm::mat4(1.0f);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            glmR[i][j] = rot(i, j);
        }
    }
    return glmR;
}

void RigidBody::step(double h, std::vector<std::shared_ptr<IForceField>>& forceFields, SimParams& simParams) {
    Eigen::Vector3d fNet;
    for (const auto & forceField : forceFields) {
        fNet += forceField->getForce(x);
    }
    Eigen::Vector3d acc = fNet / mass;
    x += h * v;
    v += h * acc;

    Eigen::Matrix3d wstar;
    wstar << 0, -angV.z(), angV.y(),
            angV.z(), 0, -angV.x(),
            -angV.y(), angV.x(), 0;
    R += h * (wstar * R);
    R.col(0).normalize();
    R.col(1).normalize();
    R.col(2).normalize();
//    std::cout<<"x: "<<x.transpose()<<"\n";

    for (int i = 0; i < polygons.size(); ++i) {
        for (int j = 0; j < polygons[i].points.size(); ++j) {
            polygons[i].points[j] =  (R * polygons0[i].points[j]) + x;
        }
    }

}

std::vector<Polygon>& RigidBody::getPolygons() {
    return polygons;
}

void RigidBody::loadMesh(const string &meshName)
{
    // Load geometry
    meshFilename = meshName;
    loadObj(meshFilename, posBuf, norBuf, texBuf);
    std::cout<<"Loaded obj\n";
}

void RigidBody::setObstacle(bool isObstacle) {
    this->isObstacle = isObstacle;
}

bool RigidBody::getObstacle() {
    return isObstacle;
}

void RigidBody::init()
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

void RigidBody::cleanupBuffers() {
    glDeleteBuffers(1, &posBufID);
    glDeleteBuffers(1, &norBufID);
    glDeleteBuffers(1, &texBufID);
    glDeleteVertexArrays(1, &VAO);
}

void RigidBody::draw(std::shared_ptr<MatrixStack> MV, const std::shared_ptr<Program> p)
{
    this->setProgram(p);
    glUniform3f(prog->getUniform("ka"), 0.1f, 0.1f, 0.1f);
    glUniform3f(prog->getUniform("ks"), 0.1f, 0.1f, 0.1f);
    glUniform1f(prog->getUniform("s"), 200.0f);
    MV->pushMatrix();
    MV->translate(x(0), x(1), x(2));
    MV->multMatrix(convertToGLMMat(R));
    MV->scale(1.0);

    glUniformMatrix4fv(prog->getUniform("MV"), 1, GL_FALSE, glm::value_ptr(MV->topMatrix()));

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

    MV->popMatrix();
}
