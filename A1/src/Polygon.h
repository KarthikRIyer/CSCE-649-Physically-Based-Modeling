//
// Created by Karthik Iyer on 01/09/24.
//

#ifndef HAIRSIM_POLYGON_H
#define HAIRSIM_POLYGON_H

#include <vector>

#include <Eigen/Dense>

class Polygon {
public:
    Polygon();
    void addPoint(float x, float y, float z);

    std::vector<Eigen::Vector3d> points;
};


#endif //HAIRSIM_POLYGON_H
