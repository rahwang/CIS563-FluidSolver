//
//  grid.hpp
//  Thanda

#ifndef grid_hpp
#define grid_hpp

#include "../geom/particles.hpp"
#include <iostream>

#define MATHIF(test, if_true, if_false) (((if_true) * (test)) + ((if_false) * (1 - (test))))

template <class T>
class Grid {
private:
    // The velocity or pressure quantities stored on the grid.
    std::vector<T> cells;
    // The number of particles influencing this quanitity.
    std::vector<int> particleCount;
public:
    int x_dim;
    int y_dim;
    int z_dim;

    Grid() {}
    Grid(int x, int y, int z) : x_dim(x), y_dim(y), z_dim(z) {
        cells = std::vector<T>(x_dim*y_dim*z_dim, 0.f);
        particleCount = std::vector<int>(x_dim*y_dim*z_dim, 0);
    }
    ~Grid() {}

    T& operator() (int i, int j, int k)
    {
        return cells[i + y_dim * (j + z_dim * k)];
    }

    const T& operator() (int i, int j, int k) const
    {
        return cells[i + y_dim * (j + z_dim * k)];
    }
    
    T& operator() (int i)
    {
        return cells[i];
    }
    
    const T& operator() (int i) const
    {
        return cells[i];
    }

    int getParticleCount(int i, int j, int k)
    {
        return particleCount[i + y_dim * (j + z_dim * k)];
    }

    void incrementParticleCount(int i, int j, int k)
    {
        particleCount[i + y_dim * (j + z_dim * k)]++;
    }

    void clearVelocity()
    {
        cells = std::vector<T>(x_dim*y_dim*z_dim, 0.f);
        particleCount = std::vector<int>(x_dim*y_dim*z_dim, 0);
    }
    
    void copyCells(Grid &dst_grid)
    {
        dst_grid.cells = cells;
    }

    void clear()
    {
        cells = std::vector<T>(x_dim*y_dim*z_dim, 0.f);
    }

    void printGrid()
    {
        std::cout << "START ";
        int numCells = cells.size();
        for (int i=0; i < numCells; ++i)
        {
            std::cout << cells[i] << " ";
        }
        std::cout << " END ";
    }

    void applyToCells(float f)
    {
        int numCells = cells.size();
        for (int i=0; i < numCells; ++i)
        {
            cells[i] += f;
        }
    }

    void normalizeCells()
    {
        int numCells = cells.size();
        for (int i=0; i < numCells; ++i)
        {
            cells[i] /= MATHIF(particleCount[i] > 0, particleCount[i], 1.0f);
        }
    }
    
    int getNumCells()
    {
        return cells.size();
    }

    enum {AIR = 0, FLUID = 1, SOLID = 2};
};

class MacGrid{
public:
    MacGrid() {}
    MacGrid(int x, int y, int z)
    {
        u = Grid<float>(x+1, y, z);
        v = Grid<float>(x, y+1, z);
        w = Grid<float>(x, y, z+1);

        u_old = Grid<float>(x+1, y, z);
        v_old = Grid<float>(x, y+1, z);
        w_old = Grid<float>(x, y, z+1);

        p_grid = Grid<float>(x, y, z);
        cellTypes = Grid<int>(x, y, z);
    }

    Grid<float> u;
    Grid<float> v;
    Grid<float> w;
    
    Grid<float> u_old;
    Grid<float> v_old;
    Grid<float> w_old;

    Grid<float> p_grid;
    Grid<int> cellTypes;
};

#endif /* grid_hpp */
