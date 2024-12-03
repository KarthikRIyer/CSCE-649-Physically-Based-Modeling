//
// Created by Karthik Iyer on 02/12/24.
//
#pragma once
#ifndef PBD_GRAINS_SPATIALHASH_H
#define PBD_GRAINS_SPATIALHASH_H

#include <vector>

class Particle;

// Spatial Hash based on implementation by Matthias Muller
// https://github.com/matthias-research/pages/blob/master/tenMinutePhysics/11-hashing.html

class SpatialHash {
public:
    SpatialHash();
    SpatialHash(double spacing, int maxNumberOfParticles);
    void create(std::vector<std::shared_ptr<Particle>>& particles);
    void query(std::vector<std::shared_ptr<Particle>>& particles, int i, double maxDist);
    int getQuerySize();
    std::vector<int>& getQueryIds();
private:
    int hashCoords(int xi, int yi, int zi);
    int intCoord(double coord);
    int hashPos(std::vector<std::shared_ptr<Particle>>& particles, int i);

    double spacing;
    int tableSize;
    std::vector<int> cellStart;
    std::vector<int> cellEntries;
    std::vector<int> queryIds;
    int querySize;
};


#endif //PBD_GRAINS_SPATIALHASH_H
