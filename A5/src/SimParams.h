//
// Created by Karthik Iyer on 04/12/23.
//

#ifndef A1_SIMPARAMS_H
#define A1_SIMPARAMS_H

struct SimParams {
    float restitutionCoefficient = 0.6;
    float timestep = 5e-4;
    int integrationMethod = 0;
};

#endif //A1_SIMPARAMS_H
