//
//  fluidSolver.cpp
//  Thanda

//#define DEBUG

#include "fluidSolver.hpp"
#include <iostream>
#include <tbb/tbb.h>
#include <Eigen/Sparse>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/IterativeLinearSolvers>

#define GRAVITY -9.8f
#define TIME_STEP (1.0f/100.f)
#define RES 2
#define CELL_WIDTH (1.f/RES)
#define DENSITY 1.f
#define PARTICLES_PER_CELL 1

//#define TBB 1
//#define RANDOM_SEED 1

int iter = 0;

// Be consistent with splatting/trilinear
// Ceiling clamp for floating point errs
// Dimension checking (RES)


void FlipSolver::step()
{
    // Reset grid values.
    clearGrids();
    //setSolidCells();
    // Transfer particle velocities to grid (Also marks cell types).
    storeParticleVelocityToGrid();
    
    // Copy old velocity
    macgrid.u.copyCells(macgrid.u_old);
    macgrid.v.copyCells(macgrid.v_old);
    macgrid.w.copyCells(macgrid.w_old);
    
    // Zero velocities pointing into solid cells.
    applyGravity();
    enforceBoundaryConditions();
    
    computePressure();
    extrapolateVelocity();
    enforceBoundaryConditions();
    // Update particle velocities.
    gridVelocityToParticle();
    updateParticlePositions();
    handleCollisions();
#ifdef DEBUG
    iter++;
    std::cout << "ITER: " << iter << std::endl;
    checkTypes();
#endif
}


void FlipSolver::checkTypes()
{
    int count = 0;
    for (int i=0; i < macgrid.marker.x_dim; ++i)
    {
        for (int j=0; j < macgrid.marker.y_dim; ++j)
        {
            for (int k=0; k < macgrid.marker.z_dim; ++k)
            {
                if (macgrid.marker(i, j, k) == macgrid.marker.FLUID)
                    std::cout << i << " " << j << " " << k << std::endl;
            }
        }
    }
}


bool FlipSolver::InBounds(int i, int j, int k)
{
    if (i > 0 && i < macgrid.marker.x_dim
        && j > 0 && j < macgrid.marker.y_dim
        && k > 0 && k < macgrid.marker.z_dim) {
        return true;
    }
    return false;
}


void FlipSolver::assemblePressureSolveCoefficients(std::vector<Tri> &coefficients)
{
    float scale = TIME_STEP / (DENSITY * CELL_WIDTH * CELL_WIDTH);
    // Index between 1 and dim-1 because we know the walls of our tank are solid.
    for (int i=0; i < macgrid.marker.x_dim; ++i)
    {
        for (int j=0; j < macgrid.marker.y_dim; ++j)
        {
            for (int k=0; k < macgrid.marker.z_dim; ++k)
            {
                // This is a count of the EMPTY | FLUID neighboring cells.
                if (macgrid.marker(i, j, k) == macgrid.marker.FLUID) {
                    int count = 0;
                    int curr_idx = macgrid.marker.flatIdx(i, j, k);
                    
                    int right_mark = macgrid.marker(i+1, j, k);
                    int left_mark = macgrid.marker(i-1, j, k);
                    int up_mark = macgrid.marker(i, j+1, k);
                    int down_mark = macgrid.marker(i, j-1, k);
                    int front_mark = macgrid.marker(i, j, k+1);
                    int behind_mark = macgrid.marker(i, j, k-1);

                    
                    // RIGHT
                    if (right_mark == macgrid.marker.AIR)
                    {
                        count++;
                    }
                    else if (right_mark == macgrid.marker.FLUID)
                    {
                        count++;
                        coefficients.push_back(Tri(curr_idx,
                                                   macgrid.marker.flatIdx(i+1, j, k),
                                                   -scale));
                    }
                    
                    // LEFT
                    if (left_mark == macgrid.marker.AIR)
                    {
                        count++;
                    }
                    else if (left_mark == macgrid.marker.FLUID)
                    {
                        count++;
                        coefficients.push_back(Tri(curr_idx,
                                                   macgrid.marker.flatIdx(i-1, j, k),
                                                   -scale));
                    }
                    
                    // UP
                    if (up_mark == macgrid.marker.AIR)
                    {
                        count++;
                    }
                    else if (up_mark == macgrid.marker.FLUID)
                    {
                        count++;
                        coefficients.push_back(Tri(curr_idx,
                                                   macgrid.marker.flatIdx(i, j+1, k),
                                                   -scale));
                    }
                    
                    // DOWN
                    if (down_mark == macgrid.marker.AIR)
                    {
                        count++;
                    }
                    else if (down_mark == macgrid.marker.FLUID)
                    {
                        count++;
                        coefficients.push_back(Tri(curr_idx,
                                                   macgrid.marker.flatIdx(i, j-1, k),
                                                   -scale));
                    }
                    
                    // FRONT
                    if (front_mark == macgrid.marker.AIR)
                    {
                        count++;
                    }
                    else if (front_mark == macgrid.marker.FLUID)
                    {
                        count++;
                        coefficients.push_back(Tri(curr_idx,
                                                   macgrid.marker.flatIdx(i, j, k+1),
                                                    -scale));
                    }
                    
                    // BEHIND
                    if (behind_mark == macgrid.marker.AIR)
                    {
                        count++;
                    }
                    else if (behind_mark == macgrid.marker.FLUID)
                    {
                        count++;
                        coefficients.push_back(Tri(curr_idx,
                                                   macgrid.marker.flatIdx(i, j, k-1),
                                                   -scale));
                    }
                    coefficients.push_back(Tri(curr_idx, curr_idx, count * scale));
                }
            }
        }
    }
}



void FlipSolver::computePressure()
{
    int n = macgrid.marker.getNumCells();

    Eigen::SparseMatrix<double> A(n, n);
    Eigen::VectorXd b(n);
    Eigen::VectorXd p(n);
    A.setZero();
    p.setZero();
    b.setZero();
    
    // Build b vector of divergences
    std::vector<Tri> divs;
    float scale = 1.0f / CELL_WIDTH;
    for (int i=0; i < macgrid.marker.x_dim; ++i)
    {
        for (int j=0; j < macgrid.marker.y_dim; ++j)
        {
            for (int k=0; k < macgrid.marker.z_dim; ++k)
            {
                if (macgrid.marker(i, j, k) == macgrid.marker.FLUID)
                {
                    int idx = macgrid.marker.flatIdx(i, j, k);
                    float u_div = macgrid.u(i+1, j, k) - macgrid.u(i, j, k);
                    float v_div = macgrid.v(i, j+1, k) - macgrid.v(i, j, k);
                    float w_div = macgrid.w(i, j, k+1) - macgrid.w(i, j, k);
                    float div = -scale * (u_div+v_div+w_div);
                    b(idx) = div;
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
    Eigen::ConjugateGradient<Eigen::SparseMatrix<double>, Eigen::Lower|Eigen::Upper> con(A);
    p = con.solve(b);         // use factorization to solve for the given right hand side
    
    // Update velocities
    scale = TIME_STEP / (DENSITY * CELL_WIDTH);
    for (int i=1; i < macgrid.marker.x_dim; ++i)
    {
        for (int j=1; j < macgrid.marker.y_dim; ++j)
        {
            for (int k=1; k < macgrid.marker.z_dim; ++k)
            {
                // Update u
                if (macgrid.marker(i-1, j, k) == macgrid.marker.FLUID
                    || macgrid.marker(i, j, k) == macgrid.marker.FLUID)
                {
                    if (macgrid.marker(i-1, j, k) == macgrid.marker.SOLID
                        || macgrid.marker(i, j, k) == macgrid.marker.SOLID)
                    {
                        macgrid.u(i, j, k) = 0.f;
                    }
                    else
                    {
                        macgrid.u(i, j, k) -= scale * (p[macgrid.marker.flatIdx(i, j, k)]
                                                       - p[macgrid.marker.flatIdx(i-1, j, k)]);
                    }
                }
                
                // Update v
                if (macgrid.marker(i, j-1, k) == macgrid.marker.FLUID
                    || macgrid.marker(i, j, k) == macgrid.marker.FLUID)
                {
                    if (macgrid.marker(i, j-1, k) == macgrid.marker.SOLID
                        || macgrid.marker(i, j, k) == macgrid.marker.SOLID)
                    {
                        macgrid.v(i, j, k) = 0.f;
                    }
                    else
                    {
                        macgrid.v(i, j, k) -= scale * (p[macgrid.marker.flatIdx(i, j, k)]
                                                       - p[macgrid.marker.flatIdx(i, j-1, k)]);
                    }
                }
                
                // Update w
                if (macgrid.marker(i, j, k-1) == macgrid.marker.FLUID
                    || macgrid.marker(i, j, k) == macgrid.marker.FLUID)
                {
                    if (macgrid.marker(i, j, k-1) == macgrid.marker.SOLID
                        || macgrid.marker(i, j, k) == macgrid.marker.SOLID)
                    {
                        macgrid.w(i, j, k) = 0.f;
                    }
                    else
                    {
                        macgrid.w(i, j, k) -= scale * (p[macgrid.marker.flatIdx(i, j, k)]
                                                       - p[macgrid.marker.flatIdx(i, j, k-1)]);
                    }
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
                    float div = 0.f;
                    int u_idx = macgrid.u.flatIdx(i, j, k);
                    int v_idx = macgrid.v.flatIdx(i, j, k);
                    int w_idx = macgrid.w.flatIdx(i, j, k);

                    float u_div = macgrid.u(i+1, j, k) - macgrid.u(i, j, k);
                    float v_div = macgrid.v(i, j+1, k) - macgrid.v(i, j, k);
                    float w_div = macgrid.w(i, j, k+1) - macgrid.w(i, j, k);
                    float tmp = -1 * (u_div+v_div+w_div) / CELL_WIDTH;
                    div += tmp;
                    if (abs(div) > 0.1)
                    {
                        int idx = macgrid.p.flatIdx(i, j, k);
                        std::cout << u_idx << " "<< v_idx << " "<< w_idx << " " << idx <<std::endl;
                        std::cout << "LIES DIVERGENCE IS HERE : " << i << " "<< j<< " "<<k << " : "<< div<<std::endl;

                    }
                }
            }
        }
    }
}


void FlipSolver::setSolidCells()
{
    for (int i=0; i < macgrid.marker.x_dim; ++i)
    {
        for (int j=0; j < macgrid.marker.y_dim; ++j)
        {
            for (int k=0; k < macgrid.marker.z_dim; ++k)
            {
                if (i == 0 || j == 0 || k == 0 ||
                    i == macgrid.marker.x_dim-1 || j == macgrid.marker.y_dim -1 || k == macgrid.marker.z_dim-1)
                {
                    macgrid.marker(i, j, k) = macgrid.marker.SOLID;
                }
            }
        }
    }
}


void FlipSolver::extrapolateVelocityComponent(const Grid<int>& tmp, Grid<float>& grid)
{
    for (int i=0; i < tmp.x_dim; ++i)
    {
        for (int j=0; j < tmp.y_dim; ++j)
        {
            for (int k=0; k < tmp.z_dim; ++k)
            {
                float accum = 0.f;
                int count = 0;
                if (tmp(i, j, k) != macgrid.marker.FLUID)
                {
                    // RIGHT
                    if (i+1 < tmp.x_dim && tmp(i+1, j, k) == macgrid.marker.FLUID)
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
                    if (j+1 < tmp.y_dim && tmp(i, j+1, k) == macgrid.marker.FLUID)
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
                    if (k+1 < tmp.z_dim && tmp(i, j, k+1) == macgrid.marker.FLUID)
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
                        grid(i, j, k) = accum / (float) count;
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
    Grid<int> tmp_u(macgrid.marker.x_dim, macgrid.marker.y_dim, macgrid.marker.z_dim);
    Grid<int> tmp_v(macgrid.marker.x_dim, macgrid.marker.y_dim, macgrid.marker.z_dim);
    Grid<int> tmp_w(macgrid.marker.x_dim, macgrid.marker.y_dim, macgrid.marker.z_dim);

    tmp_u.clear();
    tmp_v.clear();
    tmp_w.clear();

    for (int i=1; i < macgrid.marker.x_dim; ++i)
    {
        for (int j=1; j < macgrid.marker.y_dim; ++j)
        {
            for (int k=1; k < macgrid.marker.z_dim; ++k)
            {
                if (macgrid.marker(i, j, k) == macgrid.marker.FLUID)
                {
                    tmp_u(i, j, k) = macgrid.marker.FLUID;
                    tmp_v(i, j, k) = macgrid.marker.FLUID;
                    tmp_w(i, j, k) = macgrid.marker.FLUID;
                }
                if (macgrid.marker(i-1, j, k) == macgrid.marker.FLUID)
                {
                    tmp_u(i, j, k) = macgrid.marker.FLUID;
                }
                if (macgrid.marker(i, j-1, k) == macgrid.marker.FLUID)
                {
                    tmp_v(i, j, k) = macgrid.marker.FLUID;
                }
                if (macgrid.marker(i, j, k-1) == macgrid.marker.FLUID)
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
    int size = box->particles.size();
    tbb::parallel_for(0, size, 1, [=](int i)
    {
        Particle *p = box->particles[i];
        float OFFSET_CELL_WIDTH = CELL_WIDTH + .001;
        // Check to see if p is inside bounding volume
        if (p->pos[0] < (box->min_x + CELL_WIDTH)
            || p->pos[0] > (box->max_x - CELL_WIDTH))
        {
            p->pos[0] = std::min(std::max(p->pos[0], box->min_x + OFFSET_CELL_WIDTH), box->max_x - OFFSET_CELL_WIDTH);
        }
        if (p->pos[1] < (box->min_y + CELL_WIDTH)
            || p->pos[1] > (box->max_y - CELL_WIDTH))
        {
            p->pos[1] = std::min(std::max(p->pos[1], box->min_y + OFFSET_CELL_WIDTH), box->max_y - OFFSET_CELL_WIDTH);
        }
        if (p->pos[2] < (box->min_z + CELL_WIDTH)
            || p->pos[2] > (box->max_z - CELL_WIDTH))
        {
            p->pos[2] = std::min(std::max(p->pos[2], box->min_z + OFFSET_CELL_WIDTH), box->max_z - OFFSET_CELL_WIDTH);
        }
    });
#else
    for (Particle *p : box->particles)
    {
        float OFFSET_CELL_WIDTH = CELL_WIDTH + .001;
        // Check to see if p is inside bounding volume
        if (p->pos[0] < (box->min_x + CELL_WIDTH)
            || p->pos[0] > (box->max_x - CELL_WIDTH))
        {
            p->pos[0] = std::min(std::max(p->pos[0], box->min_x + OFFSET_CELL_WIDTH), box->max_x - OFFSET_CELL_WIDTH);
        }
        if (p->pos[1] < (box->min_y + CELL_WIDTH)
            || p->pos[1] > (box->max_y - CELL_WIDTH))
        {
            p->pos[1] = std::min(std::max(p->pos[1], box->min_y + OFFSET_CELL_WIDTH), box->max_y - OFFSET_CELL_WIDTH);
        }
        if (p->pos[2] < (box->min_z + CELL_WIDTH)
            || p->pos[2] > (box->max_z - CELL_WIDTH))
        {
            p->pos[2] = std::min(std::max(p->pos[2], box->min_z + OFFSET_CELL_WIDTH), box->max_z - OFFSET_CELL_WIDTH);
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
    for (Particle *p : box->particles)
    {
        p->pos += p->velocity * TIME_STEP;
    }
}


float randomBetween(float a, float b) {
    float random = ((float) rand()) / (float) RAND_MAX;
    float diff = b - a;
    float r = random * diff;
    return a + r;
}


void FlipSolver::init()
{
    // Get dimensions of the fluid container.
    int x = int(ceil(box->x_dim));
    int y = int(ceil(box->y_dim));
    int z = int(ceil(box->z_dim));

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
    macgrid.u.clearVelocity();
    macgrid.v.clearVelocity();
    macgrid.w.clearVelocity();
    macgrid.u_old.clearVelocity();
    macgrid.v_old.clearVelocity();
    macgrid.w_old.clearVelocity();

    macgrid.marker.clear();
}


void FlipSolver::applyGravity()
{
    macgrid.v.applyToCells(GRAVITY * TIME_STEP);
}


void FlipSolver::storeParticleVelocityToGridComponent(Particle *p, Grid<float> &grid, int dim)
{
    glm::ivec3 curr_idx = getVelocityGridIndex(p->pos, dim);
    int i = curr_idx[0];
    int j = curr_idx[1];
    int k = curr_idx[2];
    int ii = i + 1;
    int jj = j + 1;
    int kk = k + 1;
    
    glm::vec3 local_pos = p->pos - glm::vec3(box->min_x, box->min_y, box->min_z);
    local_pos[(dim + 1) % 3] -= (0.5f * CELL_WIDTH);
    local_pos[(dim + 2) % 3] -= (0.5f * CELL_WIDTH);
    
    float x_uVal = std::max(0.f, (local_pos[0] - i * CELL_WIDTH) / CELL_WIDTH);
    float y_uVal = std::max(0.f, (local_pos[1] - j * CELL_WIDTH) / CELL_WIDTH);
    float z_uVal = std::max(0.f, (local_pos[2] - k * CELL_WIDTH) / CELL_WIDTH);

//    if (x_uVal < -0.01 || x_uVal > 1.01 || y_uVal < -0.01 || y_uVal > 1.01 || z_uVal < -0.01 || z_uVal > 1.01) {
//        int x = 5;
//    }
    
    // (i, j, k)
    float weight1 = (1-x_uVal) * (1-y_uVal) * (1-z_uVal);
    grid(i, j, k) = weight1 * p->velocity[dim];
    grid.incrementTotalWeight(i, j, k, weight1);

    if (kk < grid.z_dim) {
        // (i, j, kk)
        float weight2 = (1 - x_uVal) * (1 - y_uVal) * z_uVal;
        grid(i, j, kk) = weight2 * p->velocity[dim];
        grid.incrementTotalWeight(i, j, kk, weight2);
    }
    
    if (jj < grid.y_dim) {
        // (i, jj, k)
        float weight3 = (1 - x_uVal) * y_uVal * (1 - z_uVal);
        grid(i, jj, k) = weight3 * p->velocity[dim];
        grid.incrementTotalWeight(i, jj, k, weight3);
    }
    
    if (jj < grid.y_dim && kk < grid.z_dim) {
        // (i, jj, kk)
        float weight4 = (1 - x_uVal) * y_uVal * z_uVal;
        grid(i, jj, kk) = weight4 * p->velocity[dim];
        grid.incrementTotalWeight(i, jj, kk, weight4);
    }
    
    if (ii < grid.x_dim) {
        // (ii, j, k)
        float weight5 = x_uVal * (1-y_uVal) * (1-z_uVal);
        grid(ii, j, k) = weight5 * p->velocity[dim];
        grid.incrementTotalWeight(ii, j, k, weight5);
    }
    
    if (ii < grid.x_dim && kk < grid.z_dim) {
        // (ii, j, kk)
        float weight6 = x_uVal * (1-y_uVal) * z_uVal;
        grid(ii, j, kk) = weight6 * p->velocity[dim];
        grid.incrementTotalWeight(ii, j, kk, weight6);
    }
    
    if (ii < grid.x_dim && jj < grid.y_dim) {
        // (ii, jj, k)
        float weight7 = x_uVal * y_uVal * (1-z_uVal);
        grid(ii, jj, k) = weight7 * p->velocity[dim];
        grid.incrementTotalWeight(ii, jj, k, weight7);
    }
    
    if (ii < grid.x_dim && jj < grid.y_dim && kk < grid.z_dim) {
        // (ii, jj, kk)
        float weight8 = x_uVal * y_uVal * z_uVal;
        grid(ii, jj, kk) = weight8 * p->velocity[dim];
        grid.incrementTotalWeight(ii, jj, kk, weight8);
    }

    // Mark cell type
    glm::ivec3 type_idx = getGridIndex(p->pos);
    macgrid.marker(type_idx[0], type_idx[1], type_idx[2]) = macgrid.marker.FLUID;
}


void FlipSolver::storeParticleVelocityToGrid()
{
#ifdef TBB
    int size = box->particles.size();
    tbb::parallel_for(0, size, 1, [=](int i)
    {
        Particle *p = box->particles[i];
        // Do u_grid update.
        storeParticleVelocityToGridComponent(p, macgrid.u, 0);
        // Do v_grid update.
        storeParticleVelocityToGridComponent(p, macgrid.v, 1);
        // Do w_grid update.
        storeParticleVelocityToGridComponent(p, macgrid.w, 2);
    });
#else
    for (Particle *p : box->particles)
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
glm::ivec3 FlipSolver::getVelocityGridIndex(const glm::vec3 &pos, int dim)
{
    glm::vec3 p = pos;

    // Offset two of the dimensions.
    p[(dim+1)%3] -= CELL_WIDTH * 0.5;
    p[(dim+2)%3] -= CELL_WIDTH * 0.5;

    // Transform to index space by subtracting minimum bounds and dividing by grid resolution.
    int i = int((pos[0] - box->min_x) / CELL_WIDTH);
    int j = int((pos[1] - box->min_y) / CELL_WIDTH);
    int k = int((pos[2] - box->min_z) / CELL_WIDTH);
    
    // Clamp values.
    glm::ivec3 max(macgrid.marker.x_dim, macgrid.marker.y_dim, macgrid.marker.z_dim);
    max[(dim+1)%3] += 1;
    max[(dim+2)%3] += 1;
    i = std::min(std::max(i, 0), max[0]);
    j = std::min(std::max(j, 0), max[1]);
    k = std::min(std::max(k, 0), max[2]);

    return glm::ivec3(i, j, k);
}


// Return the three indices for accessing the edge corresponding to the given position.
// Use for pressure grid.
glm::ivec3 FlipSolver::getGridIndex(const glm::vec3 &pos)
{
    // Offset by box

    // Transform to index space by subtracting minimum bounds and dividing by grid resolution.
    int i = int((pos[0] - box->min_x) / CELL_WIDTH);
    int j = int((pos[1] - box->min_y) / CELL_WIDTH);
    int k = int((pos[2] - box->min_z) / CELL_WIDTH);

    // Clamp values.
    i = std::min(std::max(i, 0), macgrid.marker.x_dim - 1);
    j = std::min(std::max(j, 0), macgrid.marker.y_dim - 1);
    k = std::min(std::max(k, 0), macgrid.marker.z_dim - 1);

    return glm::ivec3(i, j, k);
}


float lerp(float p1, float p2, float u)
{
    return p2*u + p1*(1.0f-u);
}


float FlipSolver::interpolateVelocityComponent(Particle *p, const Grid<float> &grid, int dim)
{
    // Get min and max indices. (Imagine a bounding box around particle).
    glm::ivec3 curr_idx = getVelocityGridIndex(p->pos, dim);
    
    glm::vec3 cw(CELL_WIDTH);
    glm::ivec3 next_idx = getVelocityGridIndex(p->pos + cw, dim);
    int i = curr_idx[0];
    int j = curr_idx[1];
    int k = curr_idx[2];
    int ii = next_idx[0];
    int jj = next_idx[1];
    int kk = next_idx[2];
    
    glm::vec3 local_pos = p->pos - glm::vec3(box->min_x, box->min_y, box->min_z);
    local_pos[(dim + 1) % 3] -= (0.5f * CELL_WIDTH);
    local_pos[(dim + 2) % 3] -= (0.5f * CELL_WIDTH);
    
    float x_uVal = std::min(1.0f, std::max(0.f, (local_pos[0] - i * CELL_WIDTH) / CELL_WIDTH));
    float y_uVal = std::min(1.0f, std::max(0.f, (local_pos[1] - j * CELL_WIDTH) / CELL_WIDTH));
    float z_uVal = std::min(1.0f, std::max(0.f, (local_pos[2] - k * CELL_WIDTH) / CELL_WIDTH));

    // Do x direction.
    float tmp1 = MATHIF(i!=ii, lerp(grid(i, j, k), grid(ii, j, k), x_uVal), grid(i, j, k));
    float tmp2 = MATHIF(i!=ii, lerp(grid(i, jj, k), grid(ii, jj, k), x_uVal), grid(i, jj, k));
    float tmp3 = MATHIF(i!=ii, lerp(grid(i, j, kk), grid(ii, j, kk), x_uVal), grid(i, j, kk));
    float tmp4 = MATHIF(i!=ii, lerp(grid(i, jj, kk), grid(ii, jj, kk), x_uVal), grid(i, jj, kk));

    //std::cout << tmp1 << " " << tmp2 << " " << tmp3 << " " << tmp4 << std::endl;

    // Do y direction.
    float tmp5 = MATHIF(j!=jj, lerp(tmp1, tmp2, y_uVal), tmp1);
    float tmp6 = MATHIF(j!=jj, lerp(tmp3, tmp4, y_uVal), tmp3);

    // Do z direction.
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
    int size = box->particles.size();
    tbb::parallel_for(0, size, 1, [=](int i)
    {
        Particle *p = box->particles[i];
        
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
    for (Particle *p : box->particles)
    {
        // Get PIC components.
        float pic_x = interpolateVelocityComponent(p, macgrid.u, 0);
        float pic_y = interpolateVelocityComponent(p, macgrid.v, 1);
        float pic_z = interpolateVelocityComponent(p, macgrid.w, 2);
        
        // Get FLIP components. "old" grids currently contain delta velocity.
        float flip_x = p->velocity[0] + interpolateVelocityComponent(p, macgrid.u_old, 0);
        float flip_y = p->velocity[1] + interpolateVelocityComponent(p, macgrid.v_old, 1);
        float flip_z = p->velocity[2] + interpolateVelocityComponent(p, macgrid.w_old, 2);
        
        p->velocity[0] = pic_x;
        p->velocity[1] = pic_y;
        p->velocity[2] = pic_z;
//
//        p->velocity[0] = flip_x;
//        p->velocity[1] = flip_y;
//        p->velocity[2] = flip_z;

//        p->velocity[0] = 0.05f * pic_x + 0.95f * flip_x;
//        p->velocity[1] = 0.05f * pic_y + 0.95f * flip_y;
//        p->velocity[2] = 0.05f * pic_z + 0.95f * flip_z;

        //std::cout << p->velocity[1] << std::endl;
    }
#endif
}
