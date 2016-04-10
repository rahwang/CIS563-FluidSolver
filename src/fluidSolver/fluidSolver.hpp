//
//  fluidSolver.hpp
//  Thanda

#ifndef fluidSolver_hpp
#define fluidSolver_hpp

#include "../geom/particles.hpp"
#include "grid.hpp"
#include <Eigen/Sparse>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/IterativeLinearSolvers>
#include <vector>

typedef Eigen::SparseMatrix<double> SpMat; // declares a column-major sparse matrix type of double
typedef Eigen::Triplet<double> Tri;

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
  glm::vec3 getGridIndex(const glm::vec3 &pos);

  void constructMacGrid(int x, int y, int z);
  void storeParticleVelocityToGridComponent(Particle *p, Grid<float> &grid, int dim);
  void storeParticleVelocityToGrid();

  void clearGrids();

  float interpolateVelocityComponent(Particle *p, const Grid<float> &grid, int dim);
  void gridVelocityToParticle();
  void applyGravity();
  void updateParticlePositions();
  void handleCollisions();
  void enforceBoundaryConditions();
  void extrapolateVelocityComponent(const Grid<int>& tmp, Grid<float>& grid);
  void extrapolateVelocity();
  void storeDeltaVelocity(Grid<float> &old_grid, const Grid<float> &grid);
  void setSolidCells();
  void computePressure();
  void assemblePressureSolveCoefficients(std::vector<Tri> &coefficients);
  void checkTypes();

  MacGrid macgrid;
};

#endif /* fluidSolver_hpp */
