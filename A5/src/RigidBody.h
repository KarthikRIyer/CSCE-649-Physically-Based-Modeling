//
// Created by Karthik Iyer on 04/11/24.
//
#pragma once
#ifndef A5_RIGIDBODY_H
#define A5_RIGIDBODY_H

#include <memory>
#include <vector>

#define GLEW_STATIC
#include <GL/glew.h>
#include <Eigen/Core>

class MatrixStack;
class Program;
class Polygon;
class IForceField;
struct SimParams;

class RigidBody {
public:
    RigidBody(double m, Eigen::Vector3d pos, Eigen::Vector3d v, Eigen::Vector3d angV);
    virtual ~RigidBody();
    void loadObj(const std::string &filename, std::vector<float> &pos, std::vector<float> &nor, std::vector<float> &tex, bool loadNor = true, bool loadTex = true);
    void loadMesh(const std::string &meshName);
    void setProgram(std::shared_ptr<Program> p) { prog = p; }
    virtual void init();
    virtual void cleanupBuffers();
    virtual void draw(std::shared_ptr<MatrixStack> MV, const std::shared_ptr<Program> p);
    void setTextureFilename(const std::string &f) { textureFilename = f; }
    std::string getTextureFilename() const { return textureFilename; }
    std::vector<Polygon>& getPolygons();
    void setObstacle(bool isObstacle);
    bool getObstacle();
    void reset();

    void step(double h, std::vector<std::shared_ptr<IForceField>>& forceFields, SimParams& simParams);
private:
    void computeMomentOfInertia();
protected:
    double mass;

    Eigen::Vector3d x0;
    Eigen::Vector3d x;
    Eigen::Vector3d v0;
    Eigen::Vector3d v;
    Eigen::Vector3d angV0;
    Eigen::Vector3d angV;
    Eigen::Matrix3d R;
    Eigen::Matrix3d R0;

    Eigen::Matrix3d I;

    std::string meshFilename;
    std::string textureFilename;
    std::shared_ptr<Program> prog;
    std::vector<float> posBuf;
    std::vector<float> norBuf;
    std::vector<float> texBuf;
    bool isObstacle;
    std::vector<Polygon> polygons0;
    std::vector<Polygon> polygons;
    GLuint VAO;
    GLuint posBufID;
    GLuint norBufID;
    GLuint texBufID;
};


#endif //A5_RIGIDBODY_H
