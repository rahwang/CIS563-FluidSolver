//
//  geom.cpp
//  Thanda

#include "particles.hpp"

#include <iostream>

void ParticleContainer::initParticles(const glm::vec3 &bounds, const glm::vec3 &container_bounds) {
    particles.clear();

    min_bound_x = -container_bounds[0];
    min_bound_y = -container_bounds[1];
    min_bound_z = -container_bounds[2];
    max_bound_x = container_bounds[0];
    max_bound_y = container_bounds[1];
    max_bound_z = container_bounds[2];
    x_dim = max_bound_x - min_bound_x;
    y_dim = max_bound_y - min_bound_y;
    z_dim = max_bound_z - min_bound_z;

    for (float i=0.f; i < bounds[0] ; i += separation) {
        for (float j=0.f; j < bounds[1]; j += separation) {
            for (float k=0.f; k < bounds[2]; k += separation) {
                glm::vec3 pos(i, j, k);
                glm::vec3 vel = glm::vec3(0.f, -9.8f, 0.f);
                particles.push_back(new Particle(pos, vel));
                //std::cout << vel[0] << " " << vel[1] << " " << vel[2] << std::endl;
            }
        }
    }

    num_indicies = particles.size() * 3 + 1;
    //std::cout << "SIZE" << num_indicies << std::endl;
}


void ParticleContainer::detectCollisions() {

    for (Particle *p : particles) {
        p->dead = ((p->position[0] < min_bound_x || p->position[0] > max_bound_x)
                || (p->position[1] < min_bound_y || p->position[1] > max_bound_y)
                || (p->position[2] < min_bound_z || p->position[2] > max_bound_z));
    }
}

void ParticleContainer::create() {
    // Our vertices. Tree consecutive floats give a 3D vertex; Three consecutive vertices give a triangle.
    // A cube has 6 faces with 2 triangles each, so this makes 6*2=12 triangles, and 12*3 vertices
    std::vector<GLfloat> g_vertex_buffer_data;
    std::vector<GLfloat> g_color_buffer_data;

    for (Particle *p : particles) {
        g_vertex_buffer_data.push_back(p->position[0]);
        g_vertex_buffer_data.push_back(p->position[1]);
        g_vertex_buffer_data.push_back(p->position[2]);

        if (p->dead) {
            g_color_buffer_data.push_back(1.0f);
            g_color_buffer_data.push_back(1.0f);
            g_color_buffer_data.push_back(1.0f);
        } else {
            g_color_buffer_data.push_back(0.0f);
            g_color_buffer_data.push_back(0.0f);
            g_color_buffer_data.push_back(0.5f);
        }
    }

    glGenBuffers(1, &vertexbuffer);
    glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * g_vertex_buffer_data.size(), g_vertex_buffer_data.data(), GL_STATIC_DRAW);

    glGenBuffers(1, &colorbuffer);
    glBindBuffer(GL_ARRAY_BUFFER, colorbuffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * g_color_buffer_data.size(), g_color_buffer_data.data(), GL_STATIC_DRAW);

//    std::cout << g_vertex_buffer_data.size() << "<size";
}
