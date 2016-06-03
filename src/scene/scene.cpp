//
//  scene.cpp
//  Thanda

#include "scene.hpp"

#include <fstream>

void Scene::loadScene(const char *filename) {

    std::string contents;
    std::string line;
    std::ifstream myfile (filename);
    if (myfile.is_open())
    {
      while ( std::getline (myfile,line) )
      {
        contents += line;
      }
      myfile.close();
    }

    Json::Value root;   // will contains the root value after parsing.
    Json::Reader reader;
    bool parsingSuccessful = reader.parse( contents, root );
    if ( !parsingSuccessful )
    {
        // report to the user the failure and their locations in the document.
        std::cout  << "Failed to parse configuration\n"
                   << reader.getFormattedErrorMessages();
        return;
    }

    // Get fluid bounding box.
    Cube *box = new Cube();
    glm::vec3 container_bounds(root["containerDim"]["scaleX"].asFloat(),
                               root["containerDim"]["scaleY"].asFloat(),
                               root["containerDim"]["scaleZ"].asFloat());
    box->setModelMatrix(glm::scale(box->getModelMatrix(), container_bounds));
    objects.push_back(box);

    // Get particle bounds.
    ParticleContainer *fluid_particles = new ParticleContainer();
    glm::vec3 particle_bounds = glm::vec3(root["particleDim"]["boundX"].asFloat(),
                                          root["particleDim"]["boundY"].asFloat(),
                                          root["particleDim"]["boundZ"].asFloat());


    fluid_particles->separation = root["particleSeparation"].asFloat();
    fluid_particles->initParticles(particle_bounds, container_bounds);
    particles = fluid_particles;
    objects.push_back(fluid_particles);

    // Create fluid solver
    fluid_solver = FlipSolver(fluid_particles);
    fluid_solver.init();
}

void Scene::drawScene(GLuint &programID, GLuint &MatrixID, Camera &camera) {
    // Use our shader
    glUseProgram(programID);

    for (Geometry *geom : objects) {
        geom->create();
        // Model matrix : an identity matrix (model will be at the origin)
        glm::mat4 Model      =  geom->getModelMatrix();
        // Our ModelViewProjection : multiplication of our 3 matrices
        glm::mat4 MVP        = camera.getViewProj() * Model; // Remember, matrix multiplication is the other way around
        glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &MVP[0][0]);

        // 1rst attribute buffer : vertices
        glEnableVertexAttribArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, geom->vertexbuffer);
        glVertexAttribPointer(
                    0,                  // attribute. No particular reason for 0, but must match the layout in the shader.
                    3,                  // size
                    GL_FLOAT,           // type
                    GL_FALSE,           // normalized?
                    0,                  // stride
                    (void*)0            // array buffer offset
                    );

        // 2nd attribute buffer : colors
        glEnableVertexAttribArray(1);
        glBindBuffer(GL_ARRAY_BUFFER, geom->colorbuffer);
        glVertexAttribPointer(
                    1,                                // attribute. No particular reason for 1, but must match the layout in the shader.
                    3,                                // size
                    GL_FLOAT,                         // type
                    GL_FALSE,                         // normalized?
                    0,                                // stride
                    (void*)0                          // array buffer offset
                    );

        // Draw the triangle !
        glPointSize(2);
        glDrawArrays(geom->draw_type, 0, geom->num_indicies); // 12*3 indices starting at 0 -> 12 triangles

        glDisableVertexAttribArray(0);
        glDisableVertexAttribArray(1);
    }
    
    // Update particle positions
    fluid_solver.step();
}
