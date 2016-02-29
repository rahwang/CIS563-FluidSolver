//
//  geom.cpp
//  Thanda

#include "particles.hpp"

#include <iostream>

#define FPS 100.f

void Particles::initParticles(const glm::vec3 &bounds, const glm::vec3 &container_bounds) {
    positions.clear();

    minBoundX = -container_bounds[0];
    minBoundY = -container_bounds[1];
    minBoundZ = -container_bounds[2];
    maxBoundX = container_bounds[0];
    maxBoundY = container_bounds[1];
    maxBoundZ = container_bounds[2];

    for (float i=0.f; i < bounds[0] ; i += separation) {
        for (float j=0.f; j < bounds[1]; j += separation) {
            for (float k=0.f; k < bounds[2]; k += separation) {
                positions.push_back(glm::vec3(i, j, k));
                speed.push_back(glm::vec3(0.f, -9.8f, 0.f) / FPS);
                dead.push_back(0);
                std::cout << i << " " << j << " " << k << std::endl;
            }
        }
    }

    num_indicies = positions.size() * 3 + 1;
    std::cout << "SIZE" << num_indicies << std::endl;
}


void Particles::detectCollisions() {
    for (int i=0; i < positions.size(); i++) {
        if ((positions[i][0] < minBoundX || positions[i][0] > maxBoundX)
                || (positions[i][1] < minBoundY || positions[i][1] > maxBoundY)
                || (positions[i][2] < minBoundZ || positions[i][2] > maxBoundZ)) {
            dead[i] = 1;
        }
    }
}

void Particles::create() {
    // Our vertices. Tree consecutive floats give a 3D vertex; Three consecutive vertices give a triangle.
    // A cube has 6 faces with 2 triangles each, so this makes 6*2=12 triangles, and 12*3 vertices
    std::vector<GLfloat> g_vertex_buffer_data;
    std::vector<GLfloat> g_color_buffer_data;
    for (int i=0; i < positions.size(); i++) {
        g_vertex_buffer_data.push_back(positions[i][0]);
        g_vertex_buffer_data.push_back(positions[i][1]);
        g_vertex_buffer_data.push_back(positions[i][2]);

        if (dead[i]) {
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
