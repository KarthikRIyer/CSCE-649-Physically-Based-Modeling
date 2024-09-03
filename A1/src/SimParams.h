//
// Created by Karthik Iyer on 04/12/23.
//

#ifndef HAIRSIM_SIMPARAMS_H
#define HAIRSIM_SIMPARAMS_H

struct SimParams {
    float windStrength = 0.0;
    float windOscilationSpeed = 1.0;
    float airFrictionFactor = 0.01;
    float restitutionCoeff = 0.6;
    float frictionCoeff = 0.5;
    float timestep = 5e-3;
};

#endif //HAIRSIM_SIMPARAMS_H
