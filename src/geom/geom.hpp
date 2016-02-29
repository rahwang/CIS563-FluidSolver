//
//  geom.hpp
//  Thanda

#ifndef geom_hpp
#define geom_hpp

#include <glm/glm.hpp>
#include <stdio.h>
#include <string>

// Include standard headers
#include <stdio.h>
#include <stdlib.h>

// Include GLEW. Always include it before gl.h and glfw.h, since it's a bit magic.
#include <GL/glew.h>

// Include GLFW
#include <GLFW/glfw3.h>

// Include GLM
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

class Geometry
{
public:
//Constructors/destructors
    Geometry() : name("GEOMETRY") {}
//Functions
    virtual ~Geometry(){}
    virtual void create() = 0;
    virtual void detectCollisions();
    const glm::mat4& getModelMatrix() { return transform; }
    void setModelMatrix(const glm::mat4 &trans) { transform = trans; }

//Member variables
    std::string name;//Mainly used for debugging purposes
    glm::mat4 transform;
    GLuint vertexbuffer;
    GLuint colorbuffer;
    GLenum draw_type;
    int num_indicies;
};


#endif /* geom_hpp */
