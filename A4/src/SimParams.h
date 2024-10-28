//
// Created by Karthik Iyer on 04/12/23.
//

#ifndef A1_SIMPARAMS_H
#define A1_SIMPARAMS_H

struct SimParams {
    float restitutionCoefficient = 0.9;
    float frictionCoefficient = 5.0;
    float timestep = 1e-3;
    int integrationMethod = 0;
};

#endif //A1_SIMPARAMS_H
