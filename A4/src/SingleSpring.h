//
// Created by Karthik Iyer on 21/10/24.
//

#ifndef A4_SingleSpring_H
#define A4_SingleSpring_H

#include <vector>
#include <Eigen/Core>

class Particle;
class Spring;
class IForceField;
struct SimParams;
class Program;
class MatrixStack;
class Shape;
class Edge;

class SingleSpring {
public:
    SingleSpring();
    virtual ~SingleSpring();

    void tare();
    void reset();
    void step(double h, std::vector<std::shared_ptr<IForceField>>& forceFields, SimParams& simParams, std::vector<std::shared_ptr<Shape> >& shapes);
    void init();
    void draw(const std::shared_ptr<Program> p) const;
    void cleanupBuffers();
private:
    std::vector<std::shared_ptr<Particle>> particles;
    std::vector<std::shared_ptr<Spring>> springs;
    std::vector<std::shared_ptr<Edge>> edges;

    std::vector<unsigned int> eleBuf;
    std::vector<float> posBuf;
    unsigned VAO;
    unsigned eleBufID;
    unsigned posBufID;

    std::pair<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d >> getVelAcc(double h,
                                                                                     std::vector<std::shared_ptr<IForceField>>& forceFields,
                                                                                     int integrationScheme);
    void detectCollision(std::shared_ptr<Particle> particle, std::vector<std::shared_ptr<Shape> >& shapes, SimParams& simParams, double h);
    void detectEdgeCollision(std::shared_ptr<Edge> edge, std::vector<std::shared_ptr<Shape> >& shapes, SimParams& simParams, double h);
    double sgn(double x);
};


#endif //A4_SingleSpring_H
