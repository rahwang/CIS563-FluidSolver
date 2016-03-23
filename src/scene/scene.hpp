//
//  scene.hpp
//  Thanda

#include <vector>
#include <json/json.h>
#include <iostream>

#include "../geom/particles.hpp"
#include "../geom/cube.hpp"
#include "../camera/camera.hpp"
#include "../fluidSolver/fluidSolver.hpp"

class Scene
{
public:
    void loadScene(const char *filepath);
    void drawScene(GLuint &programID, GLuint &MatrixID, Camera &camera);

    std::vector<Geometry *> objects;
    ParticleContainer *particles;
    FlipSolver fluid_solver;
};
