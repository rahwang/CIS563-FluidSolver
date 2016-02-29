//
//  fluidSolver.cpp
//  Thanda


#include "fluidSolver.hpp"

void FluidSolver::updateParticles(Particles *p) {

    for (int i=0; i < p->positions.size(); i++) {
        p->positions[i] += p->speed[i];
    }
    p->detectCollisions();
}
