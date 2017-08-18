#ifndef IGL_ADDIN_DRAW_FRAME
#define IGL_ADDIN_DRAW_FRAME

#include <Eigen/Core>
#include <igl/igl_inline.h>
#include "types.h"

////paints a cylinder with radius r between points a and b
//IGL_INLINE void paintCylinder(const Vector3 &a, const Vector3 &b, const double radius);
//
////paints a cone given its bottom and top points and its bottom radius
//IGL_INLINE void paintCone(const Vector3 &bottom, const Vector3 &top, const double radius);
//
////paints an arrow between from and to with a given radius
//IGL_INLINE void paintArrow(const Vector3 &from, const Vector3 &to, double radius);

//paints a coordinate frame (3 axes x,y,z in the form of arrows colored red green and blue).
//selected: a 3-element boolean array. selected[i] = true signifies that the i-th axes is selected, in which case the painted arrow will be thicker
IGL_INLINE void paintCoordinateFrame(const Vector3&point, const double length, const double radius, const bool *selected);


#ifdef IGL_HEADER_ONLY
#include "draw_frame.cpp"
#endif

#endif /*IGL_ADDIN_DRAW_FRAME*/