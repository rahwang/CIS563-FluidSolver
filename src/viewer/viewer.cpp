//
//  viewer.cpp
//  Thanda

#include "viewer.hpp"
#include "../geom/cube.hpp"

using namespace glm;

Viewer::Viewer() : width(1024), height(768), camera() {
    Init();
}

Viewer::Viewer(int w, int h) : width{w}, height{h}, camera() {
    Init();
}

Viewer::~Viewer() {}

void Viewer::render() {
    do{

        // Clear the screen
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        if (glfwGetKey(window, GLFW_KEY_UP ) == GLFW_PRESS) {
            camera.TranslateAlongLook(.5f);
        } else if (glfwGetKey(window, GLFW_KEY_DOWN ) == GLFW_PRESS) {
            camera.TranslateAlongLook(-.5f);
        } else if (glfwGetKey(window, GLFW_KEY_LEFT ) == GLFW_PRESS) {
            camera.TranslateAlongRight(-.5f);
        } else if (glfwGetKey(window, GLFW_KEY_RIGHT ) == GLFW_PRESS) {
            camera.TranslateAlongRight(.5f);
        } else if (glfwGetKey(window, GLFW_KEY_W ) == GLFW_PRESS) {
            camera.RotateAboutRight(-10);
        } else if (glfwGetKey(window, GLFW_KEY_S ) == GLFW_PRESS) {
            camera.RotateAboutRight(10);
        } else if (glfwGetKey(window, GLFW_KEY_A ) == GLFW_PRESS) {
            camera.RotateAboutUp(-10);
        } else if (glfwGetKey(window, GLFW_KEY_D ) == GLFW_PRESS) {
            camera.RotateAboutUp(10);
        }


        scene.drawScene(programID, MatrixID, camera);
        // Swap buffers
        glfwSwapBuffers(window);
        glfwPollEvents();

    } // Check if the ESC key was pressed or the window was closed
    while( glfwGetKey(window, GLFW_KEY_ESCAPE ) != GLFW_PRESS &&
           glfwWindowShouldClose(window) == 0 );

    // Cleanup VBO and shader
    for (Geometry * geom : scene.objects) {
        glDeleteBuffers(1, &geom->vertexbuffer);
        glDeleteBuffers(1, &geom->colorbuffer);
    }
    glDeleteProgram(programID);
    glDeleteVertexArrays(1, &VertexArrayID);

    // Close OpenGL window and terminate GLFW
    glfwTerminate();
}

void Viewer::Init()
{
    // Initialise GLFW
    if( !glfwInit() )
    {
        fprintf( stderr, "Failed to initialize GLFW\n" );
        getchar();
    }

    glfwWindowHint(GLFW_SAMPLES, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // To make MacOS happy; should not be needed
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    // Open a window and create its OpenGL context
    window = glfwCreateWindow( 1024, 768, "Fluid Simulation", NULL, NULL);
    if( window == NULL ){
        fprintf( stderr, "Failed to open GLFW window. If you have an Intel GPU, they are not 3.3 compatible. Try the 2.1 version of the tutorials.\n" );
        getchar();
        glfwTerminate();
    }
    glfwMakeContextCurrent(window);

    // Initialize GLEW
    glewExperimental = true; // Needed for core profile
    if (glewInit() != GLEW_OK) {
        fprintf(stderr, "Failed to initialize GLEW\n");
        getchar();
        glfwTerminate();
    }

    // Ensure we can capture the escape key being pressed below
    glfwSetInputMode(window, GLFW_STICKY_KEYS, GL_TRUE);

    // Background
    glClearColor(0.9f, 0.9f, 0.9f, 0.0f);

    // Enable depth test
    glEnable(GL_DEPTH_TEST);
    // Accept fragment if it closer to the camera than the former one
    glDepthFunc(GL_LESS);

    glGenVertexArrays(1, &VertexArrayID);
    glBindVertexArray(VertexArrayID);

    // Create and compile our GLSL program from the shaders
    programID = LoadShaders( "../CIS563-FluidSolver/src/viewer/SimpleVertexShader.vertexshader", "../CIS563-FluidSolver/src/viewer/SimpleFragmentShader.fragmentshader" );

    // Get a handle for our "MVP" uniform
    MatrixID = glGetUniformLocation(programID, "MVP");

    scene.loadScene("../CIS563-FluidSolver/src/scene/scene.json");

    render();
}
