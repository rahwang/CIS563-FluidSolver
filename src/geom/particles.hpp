//
//  particles.hpp
//  Thanda

#ifndef particles_hpp
#define particles_hpp

#include <glm/glm.hpp>
#include <vector>
#include "geom.hpp"

class Particle
{
public:
    Particle(const glm::vec3 &pos, const glm::vec3 &vel) : pos(pos), velocity(vel) {}
    ~Particle();

    glm::vec3 pos;
    glm::vec3 velocity;
    int dead;
};

class ParticleContainer : public Geometry
{
public:
//Constructors/destructors
    ParticleContainer() {
        name = "PARTICLES";
        draw_type = GL_POINTS;
        transform = glm::mat4(1.0f);
    }
//Functions
    virtual ~ParticleContainer(){}
    virtual void create();
    void initParticles(const glm::vec3 &bounds, const glm::vec3 &container_bounds);
    virtual void detectCollisions();

    std::vector<Particle *> particles;

    float min_x;
    float min_y;
    float min_z;
    float max_x;
    float max_y;
    float max_z;
    float x_dim;
    float y_dim;
    float z_dim;

    float separation;
};

#endif /* particles_hpp */
