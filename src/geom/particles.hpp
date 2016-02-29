//
//  particles.hpp
//  Thanda

#ifndef particles_hpp
#define particles_hpp

#include <glm/glm.hpp>
#include <vector>
#include "geom.hpp"

class Particles : public Geometry
{
public:
//Constructors/destructors
    Particles() {
        name = "PARTICLES";
        draw_type = GL_POINTS;
        transform = glm::mat4(1.0f);
    }
//Functions
    virtual ~Particles(){}
    virtual void create();
    void initParticles(const glm::vec3 &bounds, const glm::vec3 &container_bounds);
    virtual void detectCollisions();

    std::vector<glm::vec3> positions;
    std::vector<glm::vec3> speed;
    std::vector<glm::vec3> colors;
    std::vector<int> dead;

    float minBoundX;
    float minBoundY;
    float minBoundZ;
    float maxBoundX;
    float maxBoundY;
    float maxBoundZ;

    float separation;
};

#endif /* particles_hpp */
