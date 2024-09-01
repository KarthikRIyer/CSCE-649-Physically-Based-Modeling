//
// Created by Karthik Iyer on 01/09/24.
//

#ifndef HAIRSIM_POLYGON_H
#define HAIRSIM_POLYGON_H

#include <vector>

#include <glm/glm.hpp>

class Polygon {
public:
    Polygon();
    void addPoint(float x, float y, float z);

    std::vector<glm::vec3> points;
};


#endif //HAIRSIM_POLYGON_H
