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

#include "RigidBody.h"
#include "GLSL.h"
#include "Program.h"
#include "Polygon.h"
#include "SimParams.h"
#include "Vertex.h"

using namespace std;
using namespace glm;

RigidBody::RigidBody() :
        prog(NULL),
        posBufID(0),
        norBufID(0),
        texBufID(0),
        isObstacle(true)
{
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
            index_offset += fv;
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

void RigidBody::draw() const
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
