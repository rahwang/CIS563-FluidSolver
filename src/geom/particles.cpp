//
//  geom.cpp
//  Thanda

#include "particles.hpp"

#include <iostream>

void ParticleContainer::initParticles(const glm::vec3 &bounds, const glm::vec3 &container_bounds) {
    particles.clear();

    min_x = -container_bounds[0];
    min_y = -container_bounds[1];
    min_z = -container_bounds[2];
    max_x = container_bounds[0];
    max_y = container_bounds[1];
    max_z = container_bounds[2];
    x_dim = max_x - min_x;
    y_dim = max_y - min_y;
    z_dim = max_z - min_z;
    
    init_min_x = -bounds[0];
    init_min_y = -bounds[1];
    init_min_z = -bounds[2];
    init_max_x = bounds[0];
    init_max_y = bounds[1];
    init_max_z = bounds[2];

    glm::vec3 centered_bounds = bounds / 2.f;
    for (float i=-centered_bounds[0]; i < centered_bounds[0]; i += separation) {
        for (float j=-centered_bounds[1]; j < centered_bounds[1]; j += separation) {
            for (float k=-centered_bounds[2]; k < centered_bounds[2]; k += separation) {
                
                glm::vec3 pos(i, j, k);
                //pos += 0.25f;
                glm::vec3 vel = glm::vec3(0.f, 0.f, 0.f);
                Particle *p = new Particle(pos, vel);
                if (fabs(i - centered_bounds[0]) >= (3.0 - separation) && fabs(k - centered_bounds[2]) >= (3.0 - separation)) {
                    p->color = glm::vec3(1.0f, 0.f, 0.f);
                }

                particles.push_back(p);
                
            }
        }
    }
    
    num_indicies = particles.size();
    //std::cout << "SIZE" << num_indicies << std::endl;
}


void ParticleContainer::detectCollisions() {

    for (Particle *p : particles) {
        p->dead = ((p->pos[0] < min_x || p->pos[0] > max_x)
                || (p->pos[1] < min_y || p->pos[1] > max_y)
                || (p->pos[2] < min_z || p->pos[2] > max_z));
    }
}

void ParticleContainer::create() {
    // Our vertices. Tree consecutive floats give a 3D vertex; Three consecutive vertices give a triangle.
    // A cube has 6 faces with 2 triangles each, so this makes 6*2=12 triangles, and 12*3 vertices
    std::vector<GLfloat> g_vertex_buffer_data;
    std::vector<GLfloat> g_color_buffer_data;

    for (Particle *p : particles) {
        g_vertex_buffer_data.push_back(p->pos[0]);
        g_vertex_buffer_data.push_back(p->pos[1]);
        g_vertex_buffer_data.push_back(p->pos[2]);

        g_color_buffer_data.push_back(p->color[0]);
        g_color_buffer_data.push_back(p->color[1]);
        g_color_buffer_data.push_back(p->color[2]);
    }

    glGenBuffers(1, &vertexbuffer);
    glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * g_vertex_buffer_data.size(), g_vertex_buffer_data.data(), GL_STATIC_DRAW);

    glGenBuffers(1, &colorbuffer);
    glBindBuffer(GL_ARRAY_BUFFER, colorbuffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * g_color_buffer_data.size(), g_color_buffer_data.data(), GL_STATIC_DRAW);

//    std::cout << g_vertex_buffer_data.size() << "<size";
}
