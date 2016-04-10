//
//  fluidSolver.cpp
//  Thanda

#include "fluidSolver.hpp"
#include <iostream>
#include <tbb/tbb.h>

#define GRAVITY -9.8f
#define TIME_STEP (1.0f/10.f)
#define RES 2
#define CELL_WIDTH (1.f/RES)
#define DAMPING 1.0f
#define DENSITY 1.f

//#define TBB 1


void FlipSolver::step()
{
    // Reset grid values.
    clearGrids();
    setSolidCells();
    // Transfer particle velocities to grid (Also marks cell types).
    storeParticleVelocityToGrid();
    // Zero velocities pointing into solid cells.
    applyGravity();
    enforceBoundaryConditions();
    computePressure();
    // TODO: Pressure step.
    enforceBoundaryConditions();
    // Handle air/fluid boundaries.
    extrapolateVelocity();
    // Update particle velocities.
    gridVelocityToParticle();
    updateParticlePositions();
    handleCollisions();
}


void FlipSolver::assemblePressureSolveCoefficients(std::vector<Tri> &coefficients)
{
    float scale = TIME_STEP / (DENSITY * CELL_WIDTH * CELL_WIDTH);
    // Index between 1 and dim-1 because we know the walls of our tank are solid.
    for (int i=1; i < macgrid.marker.x_dim-1; ++i)
    {
        for (int j=1; j < macgrid.marker.y_dim-1; ++j)
        {
            for (int k=1; k < macgrid.marker.z_dim-1; ++k)
            {
                // This is a count of the EMPTY | FLUID neighboring cells.
                if (macgrid.marker(i, j, k) == macgrid.marker.FLUID) {
                    int count = 0;
                    int curr_idx = macgrid.marker.flatIdx(i, j, k);
                    
                    // RIGHT
                    if (macgrid.marker(i+1, j, k) != macgrid.marker.SOLID)
                    {
                        count++;
                        if (macgrid.marker(i+1, j, k) == macgrid.marker.FLUID)
                        {
                        coefficients.push_back(Tri(curr_idx,
                                                   macgrid.marker.flatIdx(i+1, j, k),
                                                   -scale));
                        }
                    }
                    // LEFT
                    if (macgrid.marker(i-1, j, k) != macgrid.marker.SOLID)
                    {
                        count++;
                        if (macgrid.marker(i-1, j, k) == macgrid.marker.FLUID)
                        {
                        coefficients.push_back(Tri(curr_idx,
                                                   macgrid.marker.flatIdx(i-1, j, k),
                                                   -scale));
                        }
                    }
                    // UP
                    if (macgrid.marker(i, j+1, k) != macgrid.marker.SOLID)
                    {
                        count++;
                        if (macgrid.marker(i, j+1, k) == macgrid.marker.FLUID)
                        {
                        coefficients.push_back(Tri(curr_idx,
                                                   macgrid.marker.flatIdx(i, j+1, k),
                                                   -scale));
                        }
                    }
                    // DOWN
                    if (macgrid.marker(i, j-1, k) != macgrid.marker.SOLID)
                    {
                        count++;
                        if (macgrid.marker(i, j-1, k) == macgrid.marker.FLUID)
                        {
                        coefficients.push_back(Tri(curr_idx,
                                                   macgrid.marker.flatIdx(i, j-1, k),
                                                   -scale));
                        }
                    }
                    // FRONT
                    if (macgrid.marker(i, j, k+1) != macgrid.marker.SOLID)
                    {
                        count++;
                        if (macgrid.marker(i, j, k+1) == macgrid.marker.FLUID)
                        {
                            coefficients.push_back(Tri(curr_idx,
                                                       macgrid.marker.flatIdx(i, j, k+1),
                                                       -scale));
                        }
                    }
                    // BEHIND
                    if (macgrid.marker(i, j, k-1) != macgrid.marker.SOLID)
                    {
                        count++;
                        if (macgrid.marker(i, j, k-1) == macgrid.marker.FLUID)
                        {
                            coefficients.push_back(Tri(curr_idx,
                                                       macgrid.marker.flatIdx(i, j, k-1),
                                                       -scale));
                        }
                    }
                    coefficients.push_back(Tri(curr_idx, curr_idx, count * scale));
                }
            }
        }
    }
}



void FlipSolver::computePressure()
{
    int n = macgrid.p.getNumCells();

    SpMat A(n,n);
    Eigen::VectorXd p(n);
    Eigen::VectorXd b(n);
    A.setZero();
    p.setZero();
    b.setZero();
    
    // Build b vector of divergences
    for (int i=1; i < macgrid.p.x_dim-1; ++i)
    {
        for (int j=1; j < macgrid.p.y_dim-1; ++j)
        {
            for (int k=1; k < macgrid.p.z_dim-1; ++k)
            {
                if (macgrid.marker(i, j, k) == macgrid.marker.FLUID)
                {
                    float u_div = macgrid.u(i+1, j, k) - macgrid.u(i, j, k);
                    float v_div = macgrid.v(i, j+1, k) - macgrid.v(i, j, k);
                    float w_div = macgrid.w(i, j, k+1) - macgrid.w(i, j, k);
                    b[macgrid.p.flatIdx(i, j, k)] = -1 * (u_div+v_div+w_div) / CELL_WIDTH;
                    float tmp = -1 * (u_div+v_div+w_div) / CELL_WIDTH;
                }
            }
        }
    }

    // Assemble coeff vector to be passed to coefficient matrix.
    std::vector<Tri> coefficients;
    assemblePressureSolveCoefficients(coefficients);

    // Set the coefficient matrix
    A.setFromTriplets(coefficients.begin(), coefficients.end());
    
    // Solving for pressure
    Eigen::ConjugateGradient<Eigen::SparseMatrix<double>> con;
    con.compute(A);
    p = con.solve(b);         // use factorization to solve for the given right hand side
    
    std::cout << "Divergece" << std::endl;
    std::cout << b << std::endl;
    
    std::cout << "Coefficients" << std::endl;
    std::cout << A << std::endl;

    std::cout << "Pressure" << std::endl;
    std::cout << p << std::endl;

    // Update velocities
    float scale = TIME_STEP / (DENSITY * CELL_WIDTH);
    for (int i=1; i < macgrid.p.x_dim; ++i)
    {
        for (int j=1; j < macgrid.p.y_dim; ++j)
        {
            for (int k=1; k < macgrid.p.z_dim; ++k)
            {
                if (macgrid.marker(i-1, j, k) == macgrid.marker.FLUID || macgrid.marker(i, j, k) == macgrid.marker.FLUID)
                {
                    macgrid.u(i, j, k) -= scale * (p[macgrid.p.flatIdx(i, j, k)]
                                                   - p[macgrid.p.flatIdx(i-1, j, k)]);
                }
                if (macgrid.marker(i, j-1, k) == macgrid.marker.FLUID || macgrid.marker(i, j, k) == macgrid.marker.FLUID)
                {
                macgrid.v(i, j, k) -= scale * (p[macgrid.p.flatIdx(i, j, k)]
                                               - p[macgrid.p.flatIdx(i, j-1, k)]);
                }
                if (macgrid.marker(i, j, k-1) == macgrid.marker.FLUID || macgrid.marker(i, j, k) == macgrid.marker.FLUID)
                {
                macgrid.w(i, j, k) -= scale * (p[macgrid.p.flatIdx(i, j, k)]
                                               - p[macgrid.p.flatIdx(i, j, k-1)]);
                }
            }
        }
    }
    
    // Check resulting divergences.
    for (int i=1; i < macgrid.p.x_dim-1; ++i)
    {
        for (int j=1; j < macgrid.p.y_dim-1; ++j)
        {
            for (int k=1; k < macgrid.p.z_dim-1; ++k)
            {
                if (macgrid.marker(i, j, k) == macgrid.marker.FLUID)
                {
                    float u_div = macgrid.u(i+1, j, k) - macgrid.u(i, j, k);
                    float v_div = macgrid.v(i, j+1, k) - macgrid.v(i, j, k);
                    float w_div = macgrid.w(i, j, k+1) - macgrid.w(i, j, k);
                    b[macgrid.p.flatIdx(i, j, k)] = -1 * (u_div+v_div+w_div) / CELL_WIDTH;
                    float tmp = -1 * (u_div+v_div+w_div) / CELL_WIDTH;
                }
            }
        }
    }

    std::cout << "Divergece" << std::endl;
    std::cout << b << std::endl;

}


void FlipSolver::setSolidCells()
{
    for (int i=0; i < macgrid.p.x_dim; ++i)
    {
        for (int j=0; j < macgrid.p.y_dim; ++j)
        {
            for (int k=0; k < macgrid.p.z_dim; ++k)
            {
                if (i == 0 || j == 0 || k == 0 ||
                    i == macgrid.p.x_dim-1 || j == macgrid.p.y_dim -1 || k == macgrid.p.z_dim-1)
                {
                    macgrid.marker(i, j, k) == macgrid.marker.SOLID;
                }
            }
        }
    }
}


void FlipSolver::extrapolateVelocityComponent(const Grid<int>& tmp, Grid<float>& grid)
{
    for (int i=0; i < grid.x_dim; ++i)
    {
        for (int j=0; j < grid.y_dim; ++j)
        {
            for (int k=0; k < grid.z_dim; ++k)
            {
                float accum = 0.f;
                int count = 0;
                if (tmp(i, j, k) != macgrid.marker.FLUID)
                {
                    // RIGHT
                    if (i+1 < grid.x_dim && tmp(i+1, j, k) == macgrid.marker.FLUID)
                    {
                        accum += grid(i+1, j, k);
                        count++;
                    }
                    // LEFT
                    if (i-1 >= 0 && tmp(i-1, j, k) == macgrid.marker.FLUID)
                    {
                        accum += grid(i-1, j, k);
                        count++;
                    }
                    // UP
                    if (j+1 < grid.y_dim && tmp(i, j+1, k) == macgrid.marker.FLUID)
                    {
                        accum += grid(i, j+1, k);
                        count++;
                    }
                    // DOWN
                    if (j-1 >= 0 && tmp(i, j-1, k) == macgrid.marker.FLUID)
                    {
                        accum += grid(i, j-1, k);
                        count++;
                    }
                    // FRONT
                    if (k+1 < grid.z_dim && tmp(i, j, k+1) == macgrid.marker.FLUID)
                    {
                        accum += grid(i, j, k+1);
                        count++;
                    }
                    // BEHIND
                    if (k-1 >= 0 && tmp(i, j, k-1) == macgrid.marker.FLUID)
                    {
                        accum += grid(i, j, k-1);
                        count++;
                    }
                    // Store extrapolated velocity.
                    if (count > 0)
                        grid(i, j, k) = accum / count;
                }
            }
        }
    }
}


void FlipSolver::extrapolateVelocity()
{
    // Create temporary grids marking "near fluid" cells as fluid cells.
    // These temporary grids will hold {FLUID | AIR} all AIR cells adjacent
    // to FLUID cells in the temporary grids need extrapolation.
    Grid<int> tmp_u(macgrid.u.x_dim, macgrid.u.y_dim, macgrid.u.z_dim);
    Grid<int> tmp_v(macgrid.v.x_dim, macgrid.v.y_dim, macgrid.v.z_dim);
    Grid<int> tmp_w(macgrid.w.x_dim, macgrid.w.y_dim, macgrid.w.z_dim);

    tmp_u.clear();
    tmp_v.clear();
    tmp_w.clear();

    // Populate u temp grid.
    for (int i=1; i < macgrid.u.x_dim; ++i)
    {
        for (int j=1; j < macgrid.u.y_dim; ++j)
        {
            for (int k=1; k < macgrid.u.z_dim; ++k)
            {
                if (macgrid.marker(i, j, k) == macgrid.marker.FLUID)
                {
                    tmp_u(i, j, k) = macgrid.marker.FLUID;
                }
                else if (macgrid.marker(i-1, j, k) == macgrid.marker.FLUID)
                {
                    tmp_u(i, j, k) = macgrid.marker.FLUID;
                }
            }
        }
    }

    // Populate v temp grid.
    for (int i=1; i < macgrid.v.x_dim; ++i)
    {
        for (int j=1; j < macgrid.v.y_dim; ++j)
        {
            for (int k=1; k < macgrid.v.z_dim; ++k)
            {
                if (macgrid.marker(i, j, k) == macgrid.marker.FLUID)
                {
                    tmp_v(i, j, k) = macgrid.marker.FLUID;
                }
                else if (macgrid.marker(i, j-1, k) == macgrid.marker.FLUID)
                {
                    tmp_v(i, j, k) = macgrid.marker.FLUID;
                }
            }
        }
    }

    // Populate w temp grid.
    for (int i=1; i < macgrid.w.x_dim; ++i)
    {
        for (int j=1; j < macgrid.w.y_dim; ++j)
        {
            for (int k=1; k < macgrid.w.z_dim; ++k)
            {
                if (macgrid.marker(i, j, k) == macgrid.marker.FLUID)
                {
                    tmp_w(i, j, k) = macgrid.marker.FLUID;
                }
                else if (macgrid.marker(i, j, k-1) == macgrid.marker.FLUID)
                {
                    tmp_w(i, j, k) = macgrid.marker.FLUID;
                }
            }
        }
    }

    // Now for all non-fluid cells that border fluid in the tmp grids, extrapolate.
    extrapolateVelocityComponent(tmp_u, macgrid.u);
    extrapolateVelocityComponent(tmp_v, macgrid.v);
    extrapolateVelocityComponent(tmp_w, macgrid.w);
}


void FlipSolver::handleCollisions()
{
#ifdef TBB
    int size = particle_container->particles.size();
    tbb::parallel_for(0, size, 1, [=](int i)
    {
        Particle *p = particle_container->particles[i];
        // Check to see if p is inside bounding volume
        if (p->pos[0] < particle_container->min_x || p->pos[0] > particle_container->max_x)
        {
            p->pos[0] = std::min(std::max(p->pos[0], particle_container->min_x), particle_container->max_x);
            p->velocity[0] *= -DAMPING;
        }
        if (p->pos[1] < particle_container->min_y || p->pos[1] > particle_container->max_y)
        {
            p->pos[1] = std::min(std::max(p->pos[1], particle_container->min_y), particle_container->max_y);
            p->velocity[1] *= -DAMPING;
        }
        if (p->pos[2] < particle_container->min_z || p->pos[2] > particle_container->max_z)
        {
            p->pos[2] = std::min(std::max(p->pos[2], particle_container->min_z), particle_container->max_z);
            p->velocity[2] *= -DAMPING;
        }
    });
#else
    for (Particle *p : particle_container->particles)
    {
        // Check to see if p is inside bounding volume
        if (p->pos[0] < particle_container->min_x || p->pos[0] > particle_container->max_x)
        {
            p->pos[0] = std::min(std::max(p->pos[0], particle_container->min_x), particle_container->max_x);
            p->velocity[0] *= -DAMPING;
        }
        if (p->pos[1] < particle_container->min_y || p->pos[1] > particle_container->max_y)
        {
            p->pos[1] = std::min(std::max(p->pos[1], particle_container->min_y), particle_container->max_y);
            p->velocity[1] *= -DAMPING;
        }
        if (p->pos[2] < particle_container->min_z || p->pos[2] > particle_container->max_z)
        {
            p->pos[2] = std::min(std::max(p->pos[2], particle_container->min_z), particle_container->max_z);
            p->velocity[2] *= -DAMPING;
        }
    }
#endif
}


void FlipSolver::enforceBoundaryConditions()
{
    // Enforce for u velocity.
    for (int j=0; j < macgrid.p.y_dim; ++j)
    {
        for (int k=0; k < macgrid.p.z_dim; ++k)
        {
            macgrid.u(0, j, k) = 0.f;
            macgrid.u(1, j, k) = 0.f;
            macgrid.u(macgrid.u.x_dim-2, j, k) = 0.f;
            macgrid.u(macgrid.u.x_dim-1, j, k) = 0.f;
        }
    }

    // Enforce for v velocity.
    for (int i=0; i < macgrid.p.x_dim; ++i)
    {
        for (int k=0; k < macgrid.p.z_dim; ++k)
        {
            macgrid.v(i, 0, k) = 0.f;
            macgrid.v(i, 1, k) = 0.f;
            macgrid.v(i, macgrid.v.y_dim-2, k) = 0.f;
            macgrid.v(i, macgrid.v.y_dim-1, k) = 0.f;
        }
    }

    // Enforce for w velocity.
    for (int i=0; i < macgrid.p.x_dim; ++i)
    {
        for (int j=0; j < macgrid.p.y_dim; ++j)
        {
            macgrid.w(i, j, 0) = 0.f;
            macgrid.w(i, j, 1) = 0.f;
            macgrid.w(i, j, macgrid.w.z_dim-2) = 0.f;
            macgrid.w(i, j, macgrid.w.z_dim-1) = 0.f;
        }
    }
}



// Using forward Euler for now.
void FlipSolver::updateParticlePositions()
{
    for (Particle *p : particle_container->particles)
    {
        p->pos += p->velocity * TIME_STEP;
    }
}


void FlipSolver::init()
{
    // Get dimensions of the fluid container.
    int x = int(ceil(particle_container->x_dim));
    int y = int(ceil(particle_container->y_dim));
    int z = int(ceil(particle_container->z_dim));

    constructMacGrid(x, y, z);
    applyGravity();
}


void FlipSolver::constructMacGrid(int x, int y, int z)
{
    // Initialize the macgrid and subgrids.
    macgrid = MacGrid(x*RES, y*RES, z*RES);
}


void FlipSolver::clearGrids()
{
    macgrid.u.copyCells(macgrid.u_old);
    macgrid.v.copyCells(macgrid.v_old);
    macgrid.w.copyCells(macgrid.w_old);

    macgrid.u.clearVelocity();
    macgrid.v.clearVelocity();
    macgrid.w.clearVelocity();
    macgrid.marker.clear();
}


void FlipSolver::applyGravity()
{
    macgrid.v.applyToCells(GRAVITY * TIME_STEP);
}


void FlipSolver::storeParticleVelocityToGridComponent(Particle *p, Grid<float> &grid, int dim)
{
    glm::vec3 curr_idx = getVelocityGridIndex(p->pos, dim);
    int i = curr_idx[0];
    int j = curr_idx[1];
    int k = curr_idx[2];

    if (macgrid.marker(i, j, k) == macgrid.marker.SOLID)
    {
        return;
    }

    for (int n=0; n < 2; ++n)
    {
        for (int m=0; m < 2; ++m)
        {
            for (int o=0; o < 2; ++o)
            {
                int I = i+n;
                int J = j+m;
                int K = k+o;
                if ((I < grid.x_dim) && (J < grid.y_dim) && (K < grid.z_dim))
                {
                    // Calculate weight based on distance from cell center.
                    glm::vec3 face_pos(particle_container->min_x + I * CELL_WIDTH,
                                      particle_container->min_y + J * CELL_WIDTH,
                                      particle_container->min_z + K * CELL_WIDTH);
                    face_pos[(dim + 1) % 3] += 0.5f * CELL_WIDTH;
                    face_pos[(dim + 2) % 3] += 0.5f * CELL_WIDTH;
                    float weight = 1.f / glm::length(face_pos-p->pos);
                    
                    // Store velocity contribution and increment particle counter.
                    grid(I, J, K) += weight * p->velocity[dim];
                    grid.incrementTotalWeight(I, J, K, weight);
                }
            }
        }
    }

    // Mark cell type
    glm::vec3 type_idx = getGridIndex(p->pos);
    macgrid.marker(type_idx[0], type_idx[1], type_idx[2]) = macgrid.marker.FLUID;
}


void FlipSolver::storeParticleVelocityToGrid()
{
#ifdef TBB
    int size = particle_container->particles.size();
    tbb::parallel_for(0, size, 1, [=](int i)
    {
        Particle *p = particle_container->particles[i];
        // Do u_grid update.
        storeParticleVelocityToGridComponent(p, macgrid.u, 0);
        // Do v_grid update.
        storeParticleVelocityToGridComponent(p, macgrid.v, 1);
        // Do w_grid update.
        storeParticleVelocityToGridComponent(p, macgrid.w, 2);
    });
#else
    for (Particle *p : particle_container->particles)
    {
        // Do u_grid update.
        storeParticleVelocityToGridComponent(p, macgrid.u, 0);
        // Do v_grid update.
        storeParticleVelocityToGridComponent(p, macgrid.v, 1);
        // Do w_grid update.
        storeParticleVelocityToGridComponent(p, macgrid.w, 2);
    }
#endif
    macgrid.u.normalizeCells();
    macgrid.v.normalizeCells();
    macgrid.w.normalizeCells();
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
    int i = int(floor((pos[0] - particle_container->min_x) / CELL_WIDTH));
    int j = int(floor((pos[1] - particle_container->min_y) / CELL_WIDTH));
    int k = int(floor((pos[2] - particle_container->min_z) / CELL_WIDTH));

    // Clamp values.
    i = std::min(std::max(i, 0), int(particle_container->x_dim * RES -1));
    j = std::min(std::max(j, 0), int(particle_container->y_dim * RES -1));
    k = std::min(std::max(k, 0), int(particle_container->z_dim * RES -1));

    return glm::vec3(i, j, k);
}


float lerp(float p1, float p2, float u)
{
    return p2*u + p1*(1.0f-u);
}


float FlipSolver::interpolateVelocityComponent(Particle *p, const Grid<float> &grid, int dim)
{
    // Get min and max indices. (Imagine a bounding box around particle).
    glm::vec3 curr_idx = getVelocityGridIndex(p->pos, dim);
    glm::vec3 next_idx = getVelocityGridIndex(p->pos + CELL_WIDTH, dim);
    int i = curr_idx[0];
    int j = curr_idx[1];
    int k = curr_idx[2];
    int ii = next_idx[0];
    int jj = next_idx[1];
    int kk = next_idx[2];

    // Do x direction.
    float x_uVal = (p->pos[0] - ((i * CELL_WIDTH) + particle_container->min_x)) / CELL_WIDTH;
    float tmp1 = MATHIF(i!=ii, lerp(grid(i, j, k), grid(ii, j, k), x_uVal), grid(i, j, k));
    float tmp2 = MATHIF(i!=ii, lerp(grid(i, jj, k), grid(ii, jj, k), x_uVal), grid(i, jj, k));
    float tmp3 = MATHIF(i!=ii, lerp(grid(i, j, kk), grid(ii, j, kk), x_uVal), grid(i, j, kk));
    float tmp4 = MATHIF(i!=ii, lerp(grid(i, jj, kk), grid(ii, jj, kk), x_uVal), grid(i, jj, kk));

    //std::cout << tmp1 << " " << tmp2 << " " << tmp3 << " " << tmp4 << std::endl;

    // Do y direction.
    float y_uVal = (p->pos[1] - ((j * CELL_WIDTH) + particle_container->min_y)) / CELL_WIDTH;
    float tmp5 = MATHIF(j!=jj, lerp(tmp1, tmp2, y_uVal), tmp1);
    float tmp6 = MATHIF(j!=jj, lerp(tmp3, tmp4, y_uVal), tmp3);

    // Do z direction.
    float z_uVal = (p->pos[2] - ((k * CELL_WIDTH) + particle_container->min_z)) / CELL_WIDTH;
    float THING = MATHIF(k!=kk, lerp(tmp5, tmp6, z_uVal), tmp5);
    return THING;
}


void FlipSolver::storeDeltaVelocity(Grid<float> &old_grid, const Grid<float> &grid)
{
    int numCells = old_grid.getNumCells();
    for (int i=0; i < numCells; ++i)
    {
        old_grid(i) = grid(i) - old_grid(i);
    }
}


void FlipSolver::gridVelocityToParticle()
{
    storeDeltaVelocity(macgrid.u_old, macgrid.u);
    storeDeltaVelocity(macgrid.v_old, macgrid.v);
    storeDeltaVelocity(macgrid.w_old, macgrid.w);
    
#ifdef TBB
    int size = particle_container->particles.size();
    tbb::parallel_for(0, size, 1, [=](int i)
    {
        Particle *p = particle_container->particles[i];
        
        // Get PIC components.
        float pic_x = interpolateVelocityComponent(p, macgrid.u, 0);
        float pic_y = interpolateVelocityComponent(p, macgrid.v, 1);
        float pic_z = interpolateVelocityComponent(p, macgrid.w, 2);
        
        // Get FLIP components. "old" grids currently contain delta velocity.
        float flip_x = p->velocity[0] + interpolateVelocityComponent(p, macgrid.u_old, 0);
        float flip_y = p->velocity[1] + interpolateVelocityComponent(p, macgrid.v_old, 1);
        float flip_z = p->velocity[2] + interpolateVelocityComponent(p, macgrid.w_old, 2);
        
        p->velocity[0] = 0.05f * pic_x + 0.95f * flip_x;
        p->velocity[1] = 0.05f * pic_y + 0.95f * flip_y;
        p->velocity[2] = 0.05f * pic_z + 0.95f * flip_z;
    });
#else
    for (Particle *p : particle_container->particles)
    {
        // Get PIC components.
        float pic_x = interpolateVelocityComponent(p, macgrid.u, 0);
        float pic_y = interpolateVelocityComponent(p, macgrid.v, 1);
        float pic_z = interpolateVelocityComponent(p, macgrid.w, 2);
        
        // Get FLIP components. "old" grids currently contain delta velocity.
//        float flip_x = p->velocity[0] + interpolateVelocityComponent(p, macgrid.u_old, 0);
//        float flip_y = p->velocity[1] + interpolateVelocityComponent(p, macgrid.v_old, 1);
//        float flip_z = p->velocity[2] + interpolateVelocityComponent(p, macgrid.w_old, 2);
        
//        p->velocity[0] = 0.05f * pic_x + 0.95f * flip_x;
//        p->velocity[1] = 0.05f * pic_y + 0.95f * flip_y;
//        p->velocity[2] = 0.05f * pic_z + 0.95f * flip_z;
        
        p->velocity[0] = pic_x;
        p->velocity[1] = pic_y;
        p->velocity[2] = pic_z;


        //std::cout << p->velocity[1] << std::endl;
    }
#endif
}
