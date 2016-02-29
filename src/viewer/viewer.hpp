//
//  viewer.hpp
//  Thanda

#ifndef viewer_hpp
#define viewer_hpp

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

// Include
#include "shader.hpp"
#include "../camera/camera.hpp"
#include "../scene/scene.hpp"

using namespace glm;

class Viewer {
public:
    Viewer();
    Viewer(int width, int height);
    ~Viewer();

    void render();
    void Init();
    int width;
    int height;
    GLFWwindow* window;
    GLuint programID;
    GLuint MatrixID;
    GLuint VertexArrayID;

    Camera camera;
    Scene scene;
};

#endif /* viewer_hpp */
