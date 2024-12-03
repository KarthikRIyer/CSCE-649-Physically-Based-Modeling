//
// Created by Karthik Iyer on 04/12/23.
//

#ifndef A1_SIMPARAMS_H
#define A1_SIMPARAMS_H

struct SimParams {
    float staticFrictionCoeff = 0.3;
    float kineticFrictionCoeff = 0.35;
    float timestep = 5e-3;
};

#endif //A1_SIMPARAMS_H
