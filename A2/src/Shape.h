#pragma once
#ifndef SHAPE_H
#define SHAPE_H

#include <memory>
#include <vector>

#define GLEW_STATIC
#include <GL/glew.h>

class MatrixStack;
class Program;
class Polygon;
class Particle;
struct SimParams;

class Shape
{
public:
    Shape();
    Shape(bool isGenerator, int particleCount);
    Shape(bool isGenerator, int particleCount, bool isObstacle);
    virtual ~Shape();
    void loadObj(const std::string &filename, std::vector<float> &pos, std::vector<float> &nor, std::vector<float> &tex, bool loadNor = true, bool loadTex = true);
    void loadMesh(const std::string &meshName);
    void setProgram(std::shared_ptr<Program> p) { prog = p; }
    virtual void init();
    virtual void cleanupBuffers();
    virtual void draw() const;
    void setTextureFilename(const std::string &f) { textureFilename = f; }
    std::string getTextureFilename() const { return textureFilename; }
    std::vector<Polygon>& getPolygons();
    std::vector<std::shared_ptr<Particle>> generateParticles(const std::shared_ptr<Shape>& s, SimParams& simParams, double particleSize,
                                                             double startTime, double endTime, double lifetime, double h) const;
    void setObstacle(bool isObstacle);
    bool getObstacle();
protected:
    std::string meshFilename;
    std::string textureFilename;
    std::shared_ptr<Program> prog;
    std::vector<float> posBuf;
    std::vector<float> norBuf;
    std::vector<float> texBuf;
    bool isGenerator;
    bool isObstacle;
    int particleCount;
    std::vector<Polygon> polygons;
    std::vector<double> particleFractions;
    GLuint VAO;
    GLuint posBufID;
    GLuint norBufID;
    GLuint texBufID;
};

#endif
