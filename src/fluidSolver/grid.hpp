//
//  grid.hpp
//  Thanda

#ifndef grid_hpp
#define grid_hpp

#include "../geom/particles.hpp"

class Grid{
private:
    int x_dim;
    int y_dim;
    int z_dim;
public:
    Grid() {}
    Grid(int x, int y, int z) : x_dim(x), y_dim(y), z_dim(z) {
        cells = std::vector<float>(x_dim*y_dim*z_dim, 0.f);
        particleCount = std::vector<int>(x_dim*y_dim*z_dim, 0);
    }
    ~Grid() {}    
    float& operator() (int i, int j, int k);
    const float& operator() (int i, int j, int k) const;
    float& operator() (int i);
    const float& operator() (int i) const;
    int getParticleCount(int i, int j, int k);
    void incrementParticleCount(int i, int j, int k);
    void clear();

    std::vector<float> cells;
    std::vector<int> particleCount;
};

class MacGrid{
public:
    MacGrid() {}
    MacGrid(int x, int y, int z) {
        u_grid = Grid(x+1, y, z);
        v_grid = Grid(x, y+1, z);
        w_grid = Grid(x, y, z+1);
        p_grid = Grid(x, y, z);
    }

    Grid u_grid;
    Grid v_grid;
    Grid w_grid;
    Grid p_grid;
};

#endif /* grid_hpp */
