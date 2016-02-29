//
//  fluidSolver.hpp
//  Thanda

#ifndef fluidSolver_hpp
#define fluidSolver_hpp

#include "../geom/particles.hpp"

class FluidSolver{
public:
  static void updateParticles(Particles *particles);
};

#endif /* fluidSolver_hpp */
