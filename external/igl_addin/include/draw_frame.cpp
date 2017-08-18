#include "draw_frame.h"

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

#include "draw_primitives.h"
#include <Eigen/Geometry>

using namespace Eigen;

////paints a cylinder with radius r between points a and b
//IGL_INLINE void paintCylinder(const Vector3 &a, const Vector3 &b, const double radius)
//{
//	glPushMatrix();
//
//	GLUquadricObj *quadric=gluNewQuadric();          // Create A Pointer To The Quadric Object ( NEW )
//	gluQuadricNormals(quadric, GLU_SMOOTH);   // Create Smooth Normals ( NEW )
//
//	// This is the default direction for the cylinders to face in OpenGL
//	Vector3 z = Vector3(0,0,1);
//	// Get diff between two points you want cylinder along
//	Vector3 p = (a - b);
//	// Get CROSS product (the axis of rotation)
//	Vector3 t = z.cross(p);
//
//	// Get angle. LENGTH is magnitude of the vector
//	double angle = 180 / M_PI * acos ((z.dot(p)) / p.norm());
//
//	glTranslatef(b[0],b[1],b[2]);
//	glRotated(angle,t[0],t[1],t[2]);
//
//	gluQuadricOrientation(quadric,GLU_OUTSIDE);
//	gluCylinder(quadric, radius, radius, p.norm(), 15, 15);
//
//	gluDeleteQuadric(quadric);
//	glPopMatrix();
//}
//
////paints a cone given its bottom and top points and its bottom radius
//IGL_INLINE void paintCone(const Vector3 &bottom, const Vector3 &top, const double radius)
//{
//	glPushMatrix();
//
//	GLUquadricObj *quadric=gluNewQuadric();          // Create A Pointer To The Quadric Object ( NEW )
//	gluQuadricNormals(quadric, GLU_SMOOTH);   // Create Smooth Normals ( NEW )
//
//	// This is the default direction for the cylinders to face in OpenGL
//	Vector3 z = Vector3(0,0,1);
//	// Get diff between two points you want cylinder along
//	Vector3 p = (top - bottom);
//	// Get CROSS product (the axis of rotation)
//	Vector3 t = z.cross(p);
//
//	// Get angle. LENGTH is magnitude of the vector
//	double angle = 180 / M_PI * acos ((z.dot(p)) / p.norm());
//
//	glTranslatef(bottom[0],bottom[1],bottom[2]);
//	glRotated(angle,t[0],t[1],t[2]);
//
//	gluQuadricOrientation(quadric,GLU_OUTSIDE);
//	//a cone is a cylinder with the one radius set to almost 0
//	gluCylinder(quadric, radius, 1e-8, p.norm(), 15, 15);
//
//	gluDeleteQuadric(quadric);
//	glPopMatrix();
//}
//
////paints an arrow between from and to with a given radius
//IGL_INLINE void paintArrow(const Vector3 &from, const Vector3 &to, double radius)
//{
//	double length = (to - from).norm();
//	Vector3 axis = (to-from).normalized();
//	paintCylinder(from, to, radius);
//	paintCone(to, to+0.1*length*axis, radius);
//}

//paints a coordinate frame (3 axes x,y,z in the form of arrows colored red green and blue).
//selected: a 3-element boolean array. selected[i] = true signifies that the i-th axes is selected, in which case the painted arrow will be thicker
IGL_INLINE void paintCoordinateFrame(const Vector3&point, const double length, const double radius, const bool *selected)
{
	Matrix33 I = Matrix33::Identity();
	for (int i =0; i <3; ++i)
	{
		double color[3];
		color[0] = 0.;
		color[1] = 0.;
		color[2] = 0.;
		color[i] = 1.;
		glColor3dv(color);

		if(selected && selected[i])
			paintArrow(point, point + length*I.col(i), radius*2);
		else
			paintArrow(point, point + length*I.col(i), radius);
	}
}
