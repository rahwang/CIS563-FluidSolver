//
//  fluidSolver.cpp
//  Thanda

#include "fluidSolver.hpp"
#include <iostream>

#define TIME_STEP (1/10000.f)
#define RES 1
#define CELL_WIDTH 1.f/RES

void FlipSolver::step()
{
    // Reset grid values.
    clearGrids();
    // Transfer particle velocities to grid (Also marks cell types).
    storeParticleVelocityToGrid();
    // TODO: Apply forces.
    applyGravity();
    // TODO: Enforce boundary collisions.
    particle_container->detectCollisions();
    // TODO: Pressure step.
    // Update particle velocities.
    //interpolateVelocity();
    // Update particle positions.
    updateParticlePositions();
}


// Using RK2 method.
void FlipSolver::updateParticlePositions() {
    for (Particle *p : particle_container->particles)
    {
        p->position += p->velocity * TIME_STEP;
    }
}


void FlipSolver::init()
{
    // Get dimensions of the fluid container.
    int x = int(ceil(particle_container->x_dim));
    int y = int(ceil(particle_container->y_dim));
    int z = int(ceil(particle_container->z_dim));

    constructMacGrid(x, y, z);
}


void FlipSolver::constructMacGrid(int x, int y, int z)
{
    // Initialize the macgrid and subgrids.
    macgrid = MacGrid(x*RES, y*RES, z*RES);
}


void FlipSolver::clearGrids()
{
    macgrid.u_grid.clear();
    macgrid.v_grid.clear();
    macgrid.w_grid.clear();
    macgrid.cellTypes.clear();
}


void FlipSolver::applyGravity()
{
    macgrid.v_grid.applyToCells(-9.8f);
}


void FlipSolver::storeParticleVelocityToGridComponent(Particle *p, Grid<float> &grid, int dim)
{
    glm::vec3 curr_idx = getVelocityGridIndex(p->position, dim);
    int i = curr_idx[0];
    int j = curr_idx[1];
    int k = curr_idx[2];

    for (int n=-1; n < 2; ++n)
    {
        for (int m=-1; m < 2; ++m)
        {
            for (int o=-1; o < 2; ++o)
            {
                int I = i+n;
                int J = j+m;
                int K = k+o;
                if (((I >= 0 && I < particle_container->x_dim)
                        && (J >= 0 && J < particle_container->y_dim))
                        && (K >= 0 && K < particle_container->z_dim))
                {
                    grid(I, J, K) += p->velocity[dim];
                    grid.incrementParticleCount(I, J, K);
                }
            }
        }
    }

    // Mark cell type
    glm::vec3 type_idx = getGridIndex(p->position);
    macgrid.cellTypes(type_idx[0], type_idx[1], type_idx[2]) = macgrid.cellTypes.FLUID;
}


void FlipSolver::storeParticleVelocityToGrid()
{
    for (Particle *p : particle_container->particles)
    {
        // Do u_grid update.
        storeParticleVelocityToGridComponent(p, macgrid.u_grid, 0);
        // Do v_grid update.
        storeParticleVelocityToGridComponent(p, macgrid.v_grid, 1);
        // Do w_grid update.
        storeParticleVelocityToGridComponent(p, macgrid.w_grid, 2);
    }
    macgrid.u_grid.normalizeCells();
    macgrid.v_grid.normalizeCells();
    macgrid.v_grid.printGrid();
    macgrid.w_grid.normalizeCells();
}


// Return the three indices for accessing the edge corresponding to the given position.
// Dim specifies which grid. 0 for u_grid, 1 for v_grid, 2 for w_grid.
glm::vec3 FlipSolver::getVelocityGridIndex(const glm::vec3 &pos, int dim)
{
    glm::vec3 p = pos;

    // Offset two of the dimensions.
    p[(dim+1)%3] -= CELL_WIDTH * 0.5;
    p[(dim+2)%3] -= CELL_WIDTH * 0.5;

    return getGridIndex(p);
}


// Return the three indices for accessing the edge corresponding to the given position.
// Use for pressure grid.
glm::vec3 FlipSolver::getGridIndex(const glm::vec3 &pos)
{
    // Transform to index space by subtracting minimum bounds and dividing by grid resolution.
    int i = int(floor((pos[0] - particle_container->min_bound_x) / CELL_WIDTH));
    int j = int(floor((pos[1] - particle_container->min_bound_y) / CELL_WIDTH));
    int k = int(floor((pos[2] - particle_container->min_bound_z) / CELL_WIDTH));

    // Clamp values.
    i = std::min(std::max(i, 0), int(particle_container->x_dim-1));
    j = std::min(std::max(j, 0), int(particle_container->y_dim-1));
    k = std::min(std::max(k, 0), int(particle_container->z_dim-1));

    return glm::vec3(i, j, k);
}


float lerp(float p1, float p2, float u)
{
    return p1*u + p2*(1-u);
}


float FlipSolver::interpolateVelocityComponent(Particle *p, const Grid<float> &grid, int dim)
{
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
    float x_uVal = (p->position[0] - ((i * CELL_WIDTH) + particle_container->min_bound_x)) / CELL_WIDTH;
    float tmp1 = MATHIF(i!=ii, lerp(grid(i, j, k), grid(ii, j, k), x_uVal), grid(i, j, k));
    float tmp2 = MATHIF(i!=ii, lerp(grid(i, jj, k), grid(ii, jj, k), x_uVal), grid(i, jj, k));
    float tmp3 = MATHIF(i!=ii, lerp(grid(i, j, kk), grid(ii, j, kk), x_uVal), grid(i, j, kk));
    float tmp4 = MATHIF(i!=ii, lerp(grid(i, jj, kk), grid(ii, jj, kk), x_uVal), grid(i, jj, kk));

    // Do y direction.
    float y_uVal = (p->position[1] - ((j * CELL_WIDTH) + particle_container->min_bound_y)) / CELL_WIDTH;
    float tmp5 = MATHIF(j!=jj, lerp(tmp1, tmp2, y_uVal), tmp1);
    float tmp6 = MATHIF(j!=jj, lerp(tmp3, tmp4, y_uVal), tmp3);

    // Do z direction.
    float z_uVal = (p->position[2] - ((k * CELL_WIDTH) + particle_container->min_bound_z)) / CELL_WIDTH;
    return MATHIF(k!=kk, lerp(tmp5, tmp6, z_uVal), tmp5);
}

void FlipSolver::interpolateVelocity()
{
    for (Particle *p : particle_container->particles)
    {
        p->velocity[0] = interpolateVelocityComponent(p, macgrid.u_grid, 0);
        p->velocity[1] = interpolateVelocityComponent(p, macgrid.v_grid, 1);
        p->velocity[2] = interpolateVelocityComponent(p, macgrid.w_grid, 2);

        //std::cout << p->position[0] << " " << p->position[1] << " " << p->position[2];
    }
}
