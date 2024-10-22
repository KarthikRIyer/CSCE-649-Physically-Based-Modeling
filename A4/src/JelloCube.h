//
// Created by Karthik Iyer on 21/10/24.
//

#ifndef A4_JELLOCUBE_H
#define A4_JELLOCUBE_H

#include <vector>

class Particle;
class Spring;
class IForceField;
struct SimParams;
class Program;
class MatrixStack;

class JelloCube {
public:
    JelloCube();
    virtual ~JelloCube();

    void tare();
    void reset();
    void step(double h, std::vector<std::shared_ptr<IForceField>>& forceFields, SimParams& simParams);
    void init();
    void draw(const std::shared_ptr<Program> p) const;
    void cleanupBuffers();
private:
    std::vector<std::shared_ptr<Particle>> particles;
    std::vector<std::shared_ptr<Spring>> springs;

    std::vector<unsigned int> eleBuf;
    std::vector<float> posBuf;
    unsigned VAO;
    unsigned eleBufID;
    unsigned posBufID;
};


#endif //A4_JELLOCUBE_H
