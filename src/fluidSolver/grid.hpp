//
//  grid.hpp
//  Thanda

#ifndef grid_hpp
#define grid_hpp

#include "../geom/particles.hpp"
#include <iostream>
#include <cassert>

#define MATHIF(test, if_true, if_false) (((if_true) * (test)) + ((if_false) * (1 - (test))))

// #define TBB 1

template <class T>
class Grid {
private:
    // The velocity or pressure quantities stored on the grid.
    std::vector<T> cells;
    // The number of particles influencing this quanitity.
    std::vector<float> totalWeight;
public:
    int x_dim;
    int y_dim;
    int z_dim;
    int maxIndex;

    Grid() {}
    Grid(int x, int y, int z) : x_dim(x), y_dim(y), z_dim(z) {
        cells = std::vector<T>(x_dim*y_dim*z_dim, 0.f);
        totalWeight = std::vector<float>(x_dim*y_dim*z_dim, 0.f);
        maxIndex = x_dim*y_dim*z_dim;
    }
    ~Grid() {}

    T& operator() (int i, int j, int k)
    {
//        assert(i >= 0 && j >= 0 && k >= 0);
//        assert(i < x_dim && j < y_dim && k < z_dim);
//        assert(i + x_dim * (j + y_dim * k) < maxIndex);
        return cells[i + x_dim * (j + y_dim * k)];
    }

    const T& operator() (int i, int j, int k) const
    {
//        assert(i >= 0 && j >= 0 && k >= 0);
//        assert(i < x_dim && j < y_dim && k < z_dim);
//        assert(i + x_dim * (j + y_dim * k) < maxIndex);
        return cells[i + x_dim * (j + y_dim * k)];
    }
    
    T& operator() (int i)
    {
        assert(i >= 0 && i < maxIndex);
        return cells[i];
    }
    
    const T& operator() (int i) const
    {
        assert(i >= 0 && i < maxIndex);
        return cells[i];
    }

    int getParticleCount(int i, int j, int k)
    {
        assert(i >= 0 && j >= 0 && k >= 0);
        assert(i < x_dim && j < y_dim && k < z_dim);
        return totalWeight.at(i + x_dim * (j + y_dim * k));
    }

    void incrementTotalWeight(int i, int j, int k, float val)
    {
        assert(i >= 0 && j >= 0 && k >= 0);
        assert(i < x_dim && j < y_dim && k < z_dim);
        totalWeight.at(i + x_dim * (j + y_dim * k)) += val;
    }

    int flatIdx(int i, int j, int k)
    {
        return i + x_dim * (j + y_dim * k);
    }

    void clearVelocity()
    {
        std::fill(cells.begin(), cells.end(), 0.f);
        std::fill(totalWeight.begin(), totalWeight.end(), 0.f);
    }
    
    void copyCells(Grid &dst_grid)
    {
        dst_grid.cells = cells;
    }

    void clear()
    {
        std::fill(cells.begin(), cells.end(), 0);
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
#ifdef TBB
        int size = cells.size();
        tbb::parallel_for(0, size, 1, [=](int i)
        {
            cells[i] /= MATHIF(totalWeight[i] > 0, totalWeight[i], 1.0f);
        }
#else
        int numCells = cells.size();
        for (int i=0; i < numCells; ++i)
        {
            cells[i] /= MATHIF(totalWeight[i] > 0.01f, totalWeight[i], 1.0f);
        }
#endif
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

        p = Grid<float>(x, y, z);
        marker = Grid<int>(x, y, z);
    }

    Grid<float> u;
    Grid<float> v;
    Grid<float> w;
    
    Grid<float> u_old;
    Grid<float> v_old;
    Grid<float> w_old;

    Grid<float> p;
    Grid<int> marker;
};

#endif /* grid_hpp */
