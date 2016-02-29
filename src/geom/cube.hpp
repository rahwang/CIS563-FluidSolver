//
//  geom.hpp
//  Thanda

#ifndef cube_hpp
#define cube_hpp

#include "geom.hpp"

//A cube is assumed to have side lengths of 1 and a center of <0,0,0>. This means all vertices are of the form <+/-0.5, +/-0.5, +/-0.5>
//These attributes can be altered by applying a transformation matrix to the cube.
class Cube : public Geometry
{
public:
    //Constructors/destructors
    Cube()  {
        name ="CUBE";
        draw_type = GL_LINES;
        transform = glm::mat4(1.0f);
        num_indicies = 36;
    }
    //Functions
    virtual ~Cube(){}
    virtual void create();
};


#endif /* cube_hpp */
