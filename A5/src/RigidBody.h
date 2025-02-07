//
// Created by Karthik Iyer on 04/11/24.
//
#pragma once
#ifndef A5_RIGIDBODY_H
#define A5_RIGIDBODY_H

#include <memory>
#include <vector>
#include <unordered_set>

#define GLEW_STATIC
#include <GL/glew.h>
#include <Eigen/Core>
#include <glm/glm.hpp>

class MatrixStack;
class Program;
class Polygon;
class IForceField;
struct SimParams;
class Shape;

struct CollisionData {
    Eigen::Vector3d xColl;
    Eigen::Vector3d nColl;
    Eigen::Vector3d corrVec;
    Eigen::Vector3d deltaV;
    Eigen::Vector3d deltaAngV;
};

struct pair_hash {
    inline std::size_t operator()(const std::pair<int,int> & v) const {
        return v.first*31+v.second;
    }
};

class RigidBody {
public:
    RigidBody(double m, Eigen::Vector3d pos, Eigen::Vector3d v, Eigen::Vector3d angV);
    RigidBody(double m, Eigen::Vector3d pos, Eigen::Vector3d v, Eigen::Vector3d angV, Eigen::Matrix3d rotMat);
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
    void detectCollision(double h, std::vector<std::shared_ptr<Shape> >& shapes, SimParams& simParams, std::vector<Eigen::Vector3d> &c);
    void detectEdgeCollision(double h, std::vector<std::shared_ptr<Shape> >& shapes, SimParams& simParams);
    std::pair<Eigen::Vector3d, Eigen::Vector3d> getVelAcc(double h, std::vector<std::shared_ptr<IForceField>>& forceFields, int integrationScheme);
private:
    void initObjWithRot();
    void initObjWithLoc();
    void computeAABB();
    void computeMomentOfInertia();
    glm::mat4 convertToGLMMat(Eigen::Matrix3d mat);
    bool intersectsTri(Polygon p, Eigen::Vector3d pt, Eigen::Vector3d ray);
    bool pointTriCollision(double h, std::vector<std::shared_ptr<Shape> >& shapes, SimParams& simParams, std::vector<CollisionData>& collData);
    bool edgeEdgeCollision(double h, std::vector<std::shared_ptr<Shape> >& shapes, SimParams& simParams, std::vector<CollisionData>& collData);
    double sgn(double x);
protected:
    double mass;
    double massInv;

    Eigen::Vector3d x0;
    Eigen::Vector3d x;
    Eigen::Vector3d v0;
    Eigen::Vector3d v;
    Eigen::Vector3d angV0;
    Eigen::Vector3d angV;
    Eigen::Matrix3d R;
    Eigen::Matrix3d R0;

    Eigen::Matrix3d I0;
    Eigen::Matrix3d I;
    Eigen::Matrix3d I0inv;
    Eigen::Matrix3d Iinv;

    // AABB
    double xMin, xMax, yMin, yMax, zMin, zMax;

    std::string meshFilename;
    std::string textureFilename;
    std::shared_ptr<Program> prog;
    std::vector<float> posBuf;
    std::vector<float> posBuf0;
    std::vector<float> norBuf;
    std::vector<float> norBuf0;
    std::vector<float> texBuf;
    bool isObstacle;
//    std::vector<Polygon> polygons0;
//    std::vector<Polygon> polygonsTemp;
    std::vector<Polygon> polygons;
    std::unordered_set<std::pair<int, int>, pair_hash> edges;
    std::vector<Eigen::Vector3d> verticesTemp;
    std::vector<Eigen::Vector3d> vertices0;
    std::vector<Eigen::Vector3d> vertices;
    GLuint VAO;
    GLuint posBufID;
    GLuint norBufID;
    GLuint texBufID;
};


#endif //A5_RIGIDBODY_H
