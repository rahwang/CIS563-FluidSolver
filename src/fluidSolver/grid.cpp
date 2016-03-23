//
//  grid.cpp
//  Thanda


#include "grid.hpp"

float& Grid::operator() (int i, int j, int k) {
    return cells[i*x_dim*y_dim + j*y_dim + k];
}

const float& Grid::operator() (int i, int j, int k) const {
    return cells[i*x_dim*y_dim + j*y_dim + k];
}

int Grid::getParticleCount(int i, int j, int k) {
    return particleCount[i*x_dim*y_dim + j*y_dim + k];
}

void Grid::incrementParticleCount(int i, int j, int k) {
    particleCount[i*x_dim*y_dim + j*y_dim + k]++;
}

void Grid::clear() {
    cells = std::vector<float>(x_dim*y_dim*z_dim, 0.f);
    particleCount = std::vector<int>(x_dim*y_dim*z_dim, 0);
}

