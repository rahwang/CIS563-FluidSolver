//
//  fluidSolver.hpp
//  Thanda

#ifndef fluidSolver_hpp
#define fluidSolver_hpp

#include "../geom/particles.hpp"
#include "grid.hpp"

class FluidSolver{
public:
  FluidSolver() {}
  FluidSolver(ParticleContainer *particles) : particle_container(particles) {}
  virtual void init() = 0;

  ParticleContainer *particle_container = NULL;
};

class FlipSolver : public FluidSolver {
public:
  FlipSolver() {}
  FlipSolver(ParticleContainer *particles) : FluidSolver(particles) {}
  virtual void init();

  void step();

  glm::vec3 getVelocityGridIndex(const glm::vec3 &pos, int dim);
  glm::vec3 getPressureGridIndex(const glm::vec3 &pos);

  void constructMacGrid(int x, int y, int z);
  void storeParticleVelocityToGridComponent(Particle *p, Grid &grid, int dim);
  void storeParticleVelocityToGrid();

  void clearGrids();
  void normalizeGrid(Grid &grid);

  float interpolateVelocityComponent(Particle *p, const Grid &grid, int dim);
  void interpolateVelocity();

  MacGrid macgrid;
};

#endif /* fluidSolver_hpp */
