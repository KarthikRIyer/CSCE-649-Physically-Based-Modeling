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

class Shape
{
public:
    Shape();
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

protected:
    std::string meshFilename;
    std::string textureFilename;
    std::shared_ptr<Program> prog;
    std::vector<float> posBuf;
    std::vector<float> norBuf;
    std::vector<float> texBuf;
    std::vector<Polygon> polygons;
    GLuint VAO;
    GLuint posBufID;
    GLuint norBufID;
    GLuint texBufID;
};

#endif
