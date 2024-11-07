//
// Created by Karthik Iyer on 01/09/24.
//

#ifndef A1_POLYGON_H
#define A1_POLYGON_H

#include <vector>

#include <Eigen/Dense>

class Polygon {
public:
    Polygon();
    void addPoint(float x, float y, float z);
    void addPoint(int index);

    std::vector<Eigen::Vector3d> points;
    std::vector<int> vertIndices;
};


#endif //A1_POLYGON_H
