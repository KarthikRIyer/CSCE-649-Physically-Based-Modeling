//
// Created by Karthik Iyer on 02/12/24.
//

// Spatial Hash based on implementation by Matthias Muller
// https://github.com/matthias-research/pages/blob/master/tenMinutePhysics/11-hashing.html

#include "SpatialHash.h"

#include "Particle.h"

SpatialHash::SpatialHash() {

}

SpatialHash::SpatialHash(double spacing, int maxNumberOfParticles) : spacing(spacing) {
    tableSize = 2 * maxNumberOfParticles;
    cellStart = std::vector<int>(tableSize + 1, 0);
    cellEntries = std::vector<int>(maxNumberOfParticles, 0);
    queryIds = std::vector<int>(maxNumberOfParticles, 0);
    querySize = 0;
}

int SpatialHash::hashCoords(int xi, int yi, int zi) {
    int h = (xi * 92837111) ^ (yi * 689287499) ^ (zi * 283923481);
    return std::abs(h) % tableSize;
}

int SpatialHash::intCoord(double coord) {
    return std::floor(coord / spacing);
}

int SpatialHash::hashPos(std::vector<std::shared_ptr<Particle>>& particles, int i) {
    int xCoord = intCoord(particles[i]->xTemp.x());
    int yCoord = intCoord(particles[i]->xTemp.y());
    int zCoord = intCoord(particles[i]->xTemp.z());
    return hashCoords(xCoord, yCoord, zCoord);
}

int SpatialHash::getQuerySize() {
    return querySize;
}

std::vector<int>& SpatialHash::getQueryIds() {
    return queryIds;
}

void SpatialHash::create(std::vector<std::shared_ptr<Particle>>& particles) {
    int numberOfParticles = particles.size();

    // determine cell sizes
    for (int & i : cellStart) {
        i = 0;
    }

    for (int & i : cellEntries) {
        i = 0;
    }

    for (int i = 0; i < numberOfParticles; ++i) {
        int h = hashPos(particles, i);
        cellStart[h]++;
    }

    // determine cell starts

    int start = 0;
    for (int i = 0; i < tableSize; ++i) {
        start += cellStart[i];
        cellStart[i] = start;
    }
    cellStart[tableSize] = start; // guard

    // fill in objects ids
    for (int i = 0; i < numberOfParticles; ++i) {
        int h = hashPos(particles, i);
        cellStart[h]--;
        cellEntries[cellStart[h]] = i;
    }
}

void SpatialHash::query(std::vector<std::shared_ptr<Particle>>& particles, int i, double maxDist) {
    std::shared_ptr<Particle> p = particles[i];

    int x0 = intCoord(p->xTemp.x() - maxDist);
    int y0 = intCoord(p->xTemp.y() - maxDist);
    int z0 = intCoord(p->xTemp.z() - maxDist);

    int x1 = intCoord(p->xTemp.x() + maxDist);
    int y1 = intCoord(p->xTemp.y() + maxDist);
    int z1 = intCoord(p->xTemp.z() + maxDist);

    querySize = 0;

    for (int j = x0; j <= x1; ++j) {
        for (int k = y0; k <= y1 ; ++k) {
            for (int l = z0; l <= z1 ; ++l) {
                int h = hashCoords(j, k, l);
                int start = cellStart[h];
                int end = cellStart[h+1];

                for (int m = start; m < end; ++m) {
                    queryIds[querySize] = cellEntries[m];
                    querySize++;
                }
            }
        }
    }
}