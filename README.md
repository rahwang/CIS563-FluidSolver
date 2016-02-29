# CIS563-FluidSolver
(Credit : CIS565 README)

Hello!

All required functionality implemented, although I plan to go back and change my collision so that things are deleted properly as opposed to just changing color.

#Controls

UP: zoom in
DOWN: zoom out
LEFT: pan left
RIGHT: pan right
W: rotate about the camera right axis, negative direction
S: rotate about the camera right axis, positive direction
A: rotate about the camera up axis, negative direction
D: rotate about the camera up axis, positive direction

#Organization

Camera:
Stores all camera operations for matrix computation and movement.

FluidSolver:
Will eventually be library of functions to act on Particles.

Scene:
Container for scene geometry. Initializes scene with json.

Viewer:
Calls all the window operations. Draws geometry with a call from scene.

Geometry:
Abstract class defining required functionality for the subclasses... basically just the create function for now.

    Particles:
    A object storing a list of particle positions and states in a bunch of parallel vectors... I'm considering changing to having a particle object also. Also a particle init function.

    Cube:
    Essentially just used for the fluid container. Basic cube drawing.

#External code

I used the open gl tutorial code as reference. I changed most of the code, but I'm using the loadShader code in shader.cpp verbatim. This is the code for loading the shader programs, which reads in the fragement and vertex shader code, compiles it, and links it, checking for errors at each step. This essentially just does everything necessary to actually use the shaders.



