# CIS563-FluidSolver
(Credit : CIS565 README)

Hello!

This is a fluid solver and OpenGL particle viewer that I created for CIS563:Physically-Based Animation. The fluid solver uses the FLIP/PIC algorithm, which is much too complicated for me to do justice here. Instead, I link you to the original SIGGRAPH paper, a veritable masterpiece: https://www.cs.ubc.ca/~rbridson/docs/zhu-siggraph05-sandfluid.pdf

Although there's still much to be done here, I'm quite proud of this simulator -- I consider it to be some of the trickiest implementation I've ever tackled. 

Next up:
I plan to find a way to export my particles and positions to Houdini so that I can create a lego fluid effect, as depicted here: http://gfycat.com/TautExcellentGrouper, created by Ryan Guy.

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



