//
//  grid.hpp
//  Thanda

#ifndef grid_hpp
#define grid_hpp

#include "../geom/particles.hpp"
#include <iostream>

#define MATHIF(test, if_true, if_false) (((if_true) * (test)) + ((if_false) * (!test)))

template <class T>
class Grid {
private:
    int x_dim;
    int y_dim;
    int z_dim;

    // The velocity or pressure quantities stored on the grid.
    std::vector<T> cells;
    // The number of particles influencing this quanitity.
    std::vector<int> particleCount;
public:
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

    int getParticleCount(int i, int j, int k)
    {
        return particleCount[i + y_dim * (j + z_dim * k)];
    }

    void incrementParticleCount(int i, int j, int k)
    {
        particleCount[i + y_dim * (j + z_dim * k)]++;
    }

    void clear()
    {
        cells = std::vector<T>(x_dim*y_dim*z_dim, 0);
        particleCount = std::vector<int>(x_dim*y_dim*z_dim, 0);
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
            cells[i] /= MATHIF(particleCount[i] > 0, particleCount[i], 1);
        }
    }

    enum {AIR = 0, FLUID = 1, SOLID = 2};
};

class MacGrid{
public:
    MacGrid() {}
    MacGrid(int x, int y, int z) {
        u_grid = Grid<float>(x+1, y, z);
        v_grid = Grid<float>(x, y+1, z);
        w_grid = Grid<float>(x, y, z+1);
        p_grid = Grid<float>(x, y, z);
        cellTypes = Grid<int>(x, y, z);
    }

    Grid<float> u_grid;
    Grid<float> v_grid;
    Grid<float> w_grid;
    Grid<float> p_grid;
    Grid<int> cellTypes;
};

#endif /* grid_hpp */
