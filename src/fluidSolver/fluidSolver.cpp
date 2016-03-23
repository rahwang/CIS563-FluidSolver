//
//  fluidSolver.cpp
//  Thanda

#include "fluidSolver.hpp"
#include <iostream>

#define FPS 1000.f
#define RES 1
#define CELL_WIDTH 1.f/RES
#define MATHIF(test, if_true, if_false) (((if_true) * (test)) + ((if_false) * (!test)))

void FlipSolver::step() {

    // Reset grid values.
    clearGrids();
    // Transfer particle velocities to grid.
    storeParticleVelocityToGrid();
    // TODO: Apply forces.
    // TODO: Enforce boundary collisions.
    particle_container->detectCollisions();
    // TODO: Classify voxels.
    // TODO: Pressure step.
    // Update particle velocities.
    //interpolateVelocity();
    // Update particle positions.
    for (Particle *p : particle_container->particles) {
       p->position += p->velocity / FPS;
    }
}


void FlipSolver::init() {

    // Get dimensions of the fluid container.
    int x = int(ceil(particle_container->x_dim));
    int y = int(ceil(particle_container->y_dim));
    int z = int(ceil(particle_container->z_dim));

    constructMacGrid(x, y, z);
}


void FlipSolver::constructMacGrid(int x, int y, int z) {
    // Initialize the macgrid and subgrids.
    macgrid = MacGrid(x*RES, y*RES, z*RES);
}


void FlipSolver::clearGrids() {
    macgrid.u_grid.clear();
    macgrid.v_grid.clear();
    macgrid.w_grid.clear();
}


void FlipSolver::normalizeGrid(Grid &grid) {
    int numCells = grid.cells.size();
    for (int i=0; i < numCells; ++i) {
        grid.cells[i] /= MATHIF(grid.particleCount[i] > 0, grid.particleCount[i], 1);
        //std::cout << grid.particleCount[i] << " ";
    }
}


void FlipSolver::storeParticleVelocityToGridComponent(Particle *p, Grid &grid, int dim) {
    // HACK TO MAKE THIS WIP CODE WORK!
    //if (p->dead) {
    //    return;
    //}
    // END HACK.

    glm::vec3 curr_idx = getVelocityGridIndex(p->position, dim);
    int i = curr_idx[0];
    int j = curr_idx[1];
    int k = curr_idx[2];

    for (int n=0; n < 2; ++n) {
        for (int m=0; m < 2; ++m) {
            for (int o=0; o < 2; ++o) {
                int I = i+n;
                int J = j+m;
                int K = j+o;
                if (((I >= 0 && I < particle_container->x_dim)
                        && (J >= 0 && J < particle_container->y_dim))
                        && (K >= 0 && K < particle_container->z_dim)) {
                    grid(I, J, K) += p->velocity[dim];
                    grid.incrementParticleCount(I, J, K);
                }
            }
        }
    }
}


void printGrid(Grid &grid) {
    std::cout << "START ";
    int numCells = grid.cells.size();
    for (int i=0; i < numCells; ++i) {
        std::cout << grid.cells[i] << " ";
    }
    std::cout << " END ";
}


void FlipSolver::storeParticleVelocityToGrid() {
    for (Particle *p : particle_container->particles) {
        // Do u_grid update.
        storeParticleVelocityToGridComponent(p, macgrid.u_grid, 0);
        normalizeGrid(macgrid.u_grid);
        // Do v_grid update.
        storeParticleVelocityToGridComponent(p, macgrid.v_grid, 1);
        normalizeGrid(macgrid.v_grid);
        // Do w_grid update.
        storeParticleVelocityToGridComponent(p, macgrid.w_grid, 2);
        normalizeGrid(macgrid.w_grid);
    }
}


// Return the three indices for accessing the edge corresponding to the given position.
// Dim specifies which grid. 0 for u_grid, 1 for v_grid, 2 for w_grid.
glm::vec3 FlipSolver::getVelocityGridIndex(const glm::vec3 &pos, int dim) {
    glm::vec3 p = pos;

    // Offset two of the dimensions.
    p[(dim+1)%3] -= CELL_WIDTH * 0.5;
    p[(dim+2)%3] -= CELL_WIDTH * 0.5;

    // Transform to index space by subtracting minimum bounds and dividing by grid resolution.
    int i = int(floor((p[0] - particle_container->x_dim) * RES));
    int j = int(floor((p[1] - particle_container->y_dim) * RES));
    int k = int(floor((p[2] - particle_container->z_dim) * RES));

    // Clamp values.
    i = std::min(std::max(i, 0), int(particle_container->x_dim-1));
    j = std::min(std::max(j, 0), int(particle_container->y_dim-1));
    k = std::min(std::max(k, 0), int(particle_container->z_dim-1));

    return glm::vec3(i, j, k);
}


// Return the three indices for accessing the edge corresponding to the given position.
// Use for pressure grid.
glm::vec3 FlipSolver::getPressureGridIndex(const glm::vec3 &pos) {

    // Transform to index space by subtracting minimum bounds and dividing by grid resolution.
    int i = int(floor((pos[0] - particle_container->x_dim - (CELL_WIDTH * 0.5)) * RES));
    int j = int(floor((pos[1] - particle_container->y_dim - (CELL_WIDTH * 0.5)) * RES));
    int k = int(floor((pos[2] - particle_container->z_dim - (CELL_WIDTH * 0.5)) * RES));

    // Clamp values.
    i = std::min(std::max(i, 0), int(particle_container->x_dim-1));
    j = std::min(std::max(j, 0), int(particle_container->y_dim-1));
    k = std::min(std::max(k, 0), int(particle_container->z_dim-1));

    return glm::vec3(i, j, k);
}


float lerp(float p1, float p2, float u) {
    return p1*u + p2*(1-u);
}


float FlipSolver::interpolateVelocityComponent(Particle *p, const Grid &grid, int dim) {
    // Get min and max indices. (Imagine a bounding box around particle).
    glm::vec3 curr_idx = getVelocityGridIndex(p->position, dim);
    glm::vec3 next_idx = getVelocityGridIndex(p->position + CELL_WIDTH, dim);
    int i = curr_idx[0];
    int j = curr_idx[1];
    int k = curr_idx[2];
    int ii = next_idx[0];
    int jj = next_idx[1];
    int kk = next_idx[2];

    // Do x direction.
    float x_uVal = (p->position[0] - floor(p->position[0]))/ RES;
    float tmp1 = MATHIF(i!=ii, lerp(grid(i, j, k), grid(ii, j, k), x_uVal), grid(i, j, k));
    float tmp2 = MATHIF(i!=ii, lerp(grid(i, jj, k), grid(ii, jj, k), x_uVal), grid(i, jj, k));
    float tmp3 = MATHIF(i!=ii, lerp(grid(i, j, kk), grid(ii, j, kk), x_uVal), grid(i, j, kk));
    float tmp4 = MATHIF(i!=ii, lerp(grid(i, jj, kk), grid(ii, jj, kk), x_uVal), grid(i, jj, kk));

    // Do y direction.
    float y_uVal = (p->position[1] - floor(p->position[1]))/ RES;
    float tmp5 = MATHIF(j!=jj, lerp(tmp1, tmp2, y_uVal), tmp1);
    float tmp6 = MATHIF(j!=jj, lerp(tmp3, tmp4, y_uVal), tmp3);

    // Do z direction.
    float z_uVal = (p->position[2] - floor(p->position[2]))/ RES;
    return MATHIF(k!=kk, lerp(tmp5, tmp6, y_uVal), tmp5);
}

void FlipSolver::interpolateVelocity() {
    for (Particle *p : particle_container->particles) {
        p->velocity[0] = interpolateVelocityComponent(p, macgrid.u_grid, 0);
        p->velocity[1] = interpolateVelocityComponent(p, macgrid.v_grid, 1);
        p->velocity[2] = interpolateVelocityComponent(p, macgrid.w_grid, 2);

        //std::cout << p->position[0] << " " << p->position[1] << " " << p->position[2];
    }
}
