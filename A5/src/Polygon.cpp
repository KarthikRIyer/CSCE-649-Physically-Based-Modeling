//
// Created by Karthik Iyer on 01/09/24.
//

#include "Polygon.h"

Polygon::Polygon() {
    points.clear();
}

void Polygon::addPoint(float x, float y, float z) {
    points.emplace_back(x, y, z);
}

void Polygon::addPoint(int index) {
    vertIndices.push_back(index);
}
