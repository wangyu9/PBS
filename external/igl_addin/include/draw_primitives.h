#ifndef IGL_ADDIN_DRAW_PRIMITIVES
#define IGL_ADDIN_DRAW_PRIMITIVES

#include <igl/igl_inline.h>
#include <Eigen/Core>

#include "types.h"
#include <Eigen/Geometry>

#ifdef __APPLE__
#   include <OpenGL/gl.h>
#   include <OpenGL/glu.h>
#   include <GLUT/glut.h>
#else
#   ifdef _WIN32
#       include <windows.h>
#       include <GL/glew.h>
#       include <GL/glut.h>
#   endif
#   include <GL/gl.h>
#   include <GL/glu.h>
#endif

//paints a cylinder with radius r between points a and b
//template<typename DerivedV, typename Scalar>
IGL_INLINE void paintCylinder(const Vector3 &a, const Vector3 &b, const double radius);

//paints a cone given its bottom and top points and its bottom radius
IGL_INLINE void paintCone(const Vector3 &bottom, const Vector3 &top, const double radius);

//paints an arrow between from and to with a given radius
IGL_INLINE void paintArrow(const Vector3 &from, const Vector3 &to, double radius);

IGL_INLINE void draw_arrow_2d(const Eigen::Vector3d& from, const Eigen::Vector3d& to, const Eigen::Vector3d& color, const double radius);

IGL_INLINE void draw_arrow_3d(const Eigen::Vector3d& from, const Eigen::Vector3d& to, const Eigen::Vector3d& color, const double radius);

IGL_INLINE void draw_arrow(GLdouble x1, GLdouble y1, GLdouble z1, GLdouble x2, GLdouble y2, GLdouble z2, GLdouble D);

//IGL_INLINE void draw_arrow2d(const Eigen::Vector3d& from, const Eigen::Vector3d& to, const double radius);

//IGL_INLINE void draw_arrow2d(GLdouble x1, GLdouble y1, GLdouble x2, GLdouble y2, GLdouble D);

#ifdef IGL_HEADER_ONLY
#include "draw_primitives.cpp"
#endif

#endif /*IGL_ADDIN_DRAW_PRIMITIVES*/