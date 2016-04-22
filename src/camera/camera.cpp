//
//  camera.cpp
//  Thanda
//

#include "camera.hpp"

static const float PI = 3.14159265358979323846f;
static const float TWO_PI = 2 * PI;
static const float INV_PI = 1.0f / PI;
static const float DEG2RAD = PI / 180.f;
static const float RAD2DEG = 180.f / PI;

Camera::Camera():
    Camera(400, 300)
{
    look = glm::normalize(ref - eye);
    right = glm::normalize(glm::cross(look, world_up));
    up = glm::cross(right, look);
}

Camera::Camera(unsigned int w, unsigned int h):
    Camera(w, h, glm::vec3(0,0,15), glm::vec3(0,0,0), glm::vec3(0,1,0))
{}

Camera::Camera(unsigned int w, unsigned int h, const glm::vec3 &e, const glm::vec3 &r, const glm::vec3 &worldUp):
    fovy(45.f),
    width(w),
    height(h),
    near_clip(0.1f),
    far_clip(1000),
    lens_radius(0.f),
    focal_distance(-10.f),
    eye(e),
    ref(r),
    world_up(worldUp)
{
    RecomputeAttributes();
}

Camera::Camera(const Camera &c):
    fovy(c.fovy),
    width(c.width),
    height(c.height),
    near_clip(c.near_clip),
    far_clip(c.far_clip),
    lens_radius(c.lens_radius),
    focal_distance(c.focal_distance),
    aspect(c.aspect),
    eye(c.eye),
    ref(c.ref),
    look(c.look),
    up(c.up),
    right(c.right),
    world_up(c.world_up),
    V(c.V),
    H(c.H)
{}

void Camera::CopyAttributes(const Camera &c)
{
    fovy = c.fovy;
    near_clip = c.near_clip;
    far_clip = c.far_clip;
    lens_radius = c.lens_radius;
    focal_distance = c.focal_distance;
    eye = c.eye;
    ref = c.ref;
    look = c.look;
    up = c.up;
    right = c.right;
    width = c.width;
    height = c.height;
    aspect = c.aspect;
    V = c.V;
    H = c.H;
}

void Camera::RecomputeAttributes()
{
    look = glm::normalize(ref - eye);
    right = glm::normalize(glm::cross(look, world_up));
    up = glm::cross(right, look);

    float tan_fovy = tan(fovy/2 * DEG2RAD);
    float len = glm::length(ref - eye);
    aspect = float(width)/height;
    V = up*len*tan_fovy;
    H = right*len*aspect*tan_fovy;
}

glm::mat4 Camera::getViewProj()
{
    return glm::perspective(fovy, width / (float)height, near_clip, far_clip) * glm::lookAt(eye, ref, up);
}

glm::mat4 Camera::ViewMatrix()
{
    // View matrix = O * T
    glm::mat4 orientation_mat = glm::mat4(
                right.x, up.x, look.x, 0.f,
                right.y, up.y, look.y, 0.f,
                right.z, up.z, look.z, 0.f,
                0.f, 0.f, 0.f, 1.f
                );

    glm::mat4 translation_mat = glm::mat4(
                1.f, 0.f, 0.f, 0.f,
                0.f, 1.f, 0.f, 0.f,
                0.f, 0.f, 1.f, 0.f,
                -eye.x, -eye.y, -eye.z, 1.f
                );

    return orientation_mat * translation_mat;
}

glm::mat4 Camera::PerspectiveProjectionMatrix()
{
    // Compute top, bottom, left, right based on near, far, fovy, and aspect
    float top = near_clip * glm::tan(glm::radians(fovy) / 2);
    float bottom = -top;
    float right = top * aspect;
    float left = -right;

    // Build mat4 by columns
    return glm::mat4(
                (2 * near_clip) / (right - left), 0.f, 0.f, 0.f,
                0.f, (2 * near_clip) / (top - bottom), 0.f, 0.f,
                -(right + left) / (right - left), - (top + bottom) / (top - bottom), far_clip / (far_clip - near_clip), 1.f,
                0.f, 0.f, - (far_clip * near_clip)  / (far_clip - near_clip), 0.f
                );
}

void Camera::RotateAboutUp(float deg)
{
    deg *= DEG2RAD;
    glm::mat4 rotation = glm::rotate(glm::mat4(1.0f), deg, up);
    ref = ref - eye;
    ref = glm::vec3(rotation * glm::vec4(ref, 1));
    ref = ref + eye;
    RecomputeAttributes();
}
void Camera::RotateAboutRight(float deg)
{
    deg *= DEG2RAD;
    glm::mat4 rotation = glm::rotate(glm::mat4(1.0f), deg, right);
    ref = ref - eye;
    ref = glm::vec3(rotation * glm::vec4(ref, 1));
    ref = ref + eye;
    RecomputeAttributes();
}

void Camera::TranslateAlongLook(float amt)
{
    glm::vec3 translation = look * amt;
    eye += translation;
    ref += translation;
}

void Camera::TranslateAlongRight(float amt)
{
    glm::vec3 translation = right * amt;
    eye += translation;
    ref += translation;
}
void Camera::TranslateAlongUp(float amt)
{
    glm::vec3 translation = up * amt;
    eye += translation;
    ref += translation;
}

GLenum Camera::drawMode(){return GL_LINES;}
