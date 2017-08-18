#include "draw_primitives.h"

//paints a cylinder with radius r between points a and b
//template<typename DerivedV, typename Scalar>
IGL_INLINE void paintCylinder(const Vector3 &a, const Vector3 &b, const double radius)
{
	glPushMatrix();

	GLUquadricObj *quadric=gluNewQuadric();          // Create A Pointer To The Quadric Object ( NEW )
	gluQuadricNormals(quadric, GLU_SMOOTH);   // Create Smooth Normals ( NEW )

	// This is the default direction for the cylinders to face in OpenGL
	Vector3 z = Vector3(0,0,1);
	// Get diff between two points you want cylinder along
	Vector3 p = (a - b);
	// Get CROSS product (the axis of rotation)
	Vector3 t = z.cross(p);

	// Get angle. LENGTH is magnitude of the vector
	double angle = 180 / M_PI * acos ((z.dot(p)) / p.norm());

	glTranslatef(b[0],b[1],b[2]);
	glRotated(angle,t[0],t[1],t[2]);

	gluQuadricOrientation(quadric,GLU_OUTSIDE);
	gluCylinder(quadric, radius, radius, p.norm(), 15, 15);

	gluDeleteQuadric(quadric);

	glPopMatrix();
}

//paints a cone given its bottom and top points and its bottom radius
IGL_INLINE void paintCone(const Vector3 &bottom, const Vector3 &top, const double radius)
{
	glPushMatrix();

	GLUquadricObj *quadric=gluNewQuadric();          // Create A Pointer To The Quadric Object ( NEW )
	gluQuadricNormals(quadric, GLU_SMOOTH);   // Create Smooth Normals ( NEW )

	// This is the default direction for the cylinders to face in OpenGL
	Vector3 z = Vector3(0,0,1);
	// Get diff between two points you want cylinder along
	Vector3 p = (top - bottom);
	// Get CROSS product (the axis of rotation)
	Vector3 t = z.cross(p);

	// Get angle. LENGTH is magnitude of the vector
	double angle = 180 / M_PI * acos ((z.dot(p)) / p.norm());

	glTranslatef(bottom[0],bottom[1],bottom[2]);
	glRotated(angle,t[0],t[1],t[2]);

	gluQuadricOrientation(quadric,GLU_OUTSIDE);
	//a cone is a cylinder with the one radius set to almost 0
	gluCylinder(quadric, radius, 1e-8, p.norm(), 15, 15);

	gluDeleteQuadric(quadric);
	glPopMatrix();
}


//paints an arrow between from and to with a given radius
IGL_INLINE void paintArrow(const Vector3 &from, const Vector3 &to, double radius)
{
	double length = (to - from).norm();
	Vector3 axis = (to-from).normalized();
	paintCylinder(from, to, radius);
	paintCone(to, to+0.1*length*axis, radius);
}



#include <unsupported/Eigen/OpenGLSupport>

IGL_INLINE void draw_arrow_2d(const Eigen::Vector3d& from, const Eigen::Vector3d& to, const Eigen::Vector3d& color, const double radius)
{
	// Not working so fart
	//draw_arrow(from(0), from(1), from(2), to(0), to(1), to(2), radius);

	//return;

	Eigen::Vector3d p = to - from;
	double length = p.norm();
	double width = 3 * radius;
	p.normalize();
	Eigen::Vector3d n;
	n << 0, 0, 1;
	Eigen::Vector3d t = n.cross(p);

	//glPushMatrix();
	// Render flipped tets
	glDisable(GL_LIGHTING);
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_CULL_FACE);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	double pad_size = 0.3;
	//Eigen::Vector3d pad = (t + p) * width * 0.15;

	{
		glColor3f( 0, 0, 0);

		Eigen::Vector3d a = from - t * width + (-t - p) * width * pad_size;
		Eigen::Vector3d b = to - t * width + (-t + p) * width * pad_size;
		Eigen::Vector3d c = to + t * width + (t + p) * width * pad_size;
		Eigen::Vector3d d = from + t * width + (t - p) * width * pad_size;

		Eigen::Vector3d e = to - t * width * 2 + (-t * 1.5 - p) * width * pad_size;
		Eigen::Vector3d f = to + t * width * 2 + (t * 1.5 - p) * width * pad_size;
		Eigen::Vector3d g = to + p * width * 4.0 + (2 * p) * width * pad_size;

		glBegin(GL_TRIANGLES);
		glVertex(a);
		glVertex(b);
		glVertex(c);
		glEnd();

		glBegin(GL_TRIANGLES);
		glVertex(a);
		glVertex(c);
		glVertex(d);
		glEnd();

		glBegin(GL_TRIANGLES);
		glVertex(e);
		glVertex(g);
		glVertex(f);
		glEnd();
	}

	{
		glColor3f(color(0), color(1), color(2));

		Eigen::Vector3d a = from - t * width;
		Eigen::Vector3d b = to - t * width;
		Eigen::Vector3d c = to + t * width;
		Eigen::Vector3d d = from + t * width;

		Eigen::Vector3d e = to - t * width * 2;
		Eigen::Vector3d f = to + t * width * 2;
		Eigen::Vector3d g = to + p * width * 4.0;

		glBegin(GL_TRIANGLES);
		glVertex(a);
		glVertex(b);
		glVertex(c);
		glEnd();

		glBegin(GL_TRIANGLES);
		glVertex(a);
		glVertex(c);
		glVertex(d);
		glEnd();

		glBegin(GL_TRIANGLES);
		glVertex(e);
		glVertex(g);
		glVertex(f);
		glEnd();
	}




	//glPopMatrix();

	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);

}

IGL_INLINE void draw_arrow_3d(const Eigen::Vector3d& from, const Eigen::Vector3d& to, const Eigen::Vector3d& color, const double radius)
{
	glColor3f(color(0), color(1), color(2));

	glDisable(GL_LIGHTING);
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_CULL_FACE);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	paintArrow(from, to, radius);

	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);
}

#define RADPERDEG 0.0174533

IGL_INLINE void draw_arrow(GLdouble x1, GLdouble y1, GLdouble z1, GLdouble x2, GLdouble y2, GLdouble z2, GLdouble D)
{
	double x = x2 - x1;
	double y = y2 - y1;
	double z = z2 - z1;
	double L = sqrt(x*x + y*y + z*z);

	GLUquadricObj *quadObj;

	glPushMatrix();

	glTranslated(x1, y1, z1);

	if ((x != 0.) || (y != 0.)) {
		glRotated(atan2(y, x) / RADPERDEG, 0., 0., 1.);
		glRotated(atan2(sqrt(x*x + y*y), z) / RADPERDEG, 0., 1., 0.);
	}
	else if (z < 0){
		glRotated(180, 1., 0., 0.);
	}

	glTranslatef(0, 0, L - 4 * D);

	quadObj = gluNewQuadric();
	gluQuadricDrawStyle(quadObj, GLU_FILL);
	gluQuadricNormals(quadObj, GLU_SMOOTH);
	gluCylinder(quadObj, 2 * D, 0.0, 4 * D, 32, 1);
	gluDeleteQuadric(quadObj);

	quadObj = gluNewQuadric();
	gluQuadricDrawStyle(quadObj, GLU_FILL);
	gluQuadricNormals(quadObj, GLU_SMOOTH);
	gluDisk(quadObj, 0.0, 2 * D, 32, 1);
	gluDeleteQuadric(quadObj);

	glTranslatef(0, 0, -L + 4 * D);

	quadObj = gluNewQuadric();
	gluQuadricDrawStyle(quadObj, GLU_FILL);
	gluQuadricNormals(quadObj, GLU_SMOOTH);
	gluCylinder(quadObj, D, D, L - 4 * D, 32, 1);
	gluDeleteQuadric(quadObj);

	quadObj = gluNewQuadric();
	gluQuadricDrawStyle(quadObj, GLU_FILL);
	gluQuadricNormals(quadObj, GLU_SMOOTH);
	gluDisk(quadObj, 0.0, D, 32, 1);
	gluDeleteQuadric(quadObj);

	glPopMatrix();

}

//IGL_INLINE void draw_arrow2d(const Eigen::Vector3d& from, const Eigen::Vector3d& to, const double radius)
//{
//	draw_arrow2d(from(0), from(1), to(0), to(1), radius);
//}

//IGL_INLINE void draw_arrow2d(GLdouble x1, GLdouble y1, GLdouble x2, GLdouble y2, GLdouble D)
//{
//
//
//}