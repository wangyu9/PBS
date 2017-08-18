#ifndef DRAW_POINT_H
#define DRAW_POINT_H
// Draw a nice looking dot at a given point in 3d
//
// Note: expects that GL_CURRENT_COLOR is set with the desired foreground color
// 
// Inputs:
//   x  x-coordinate of point
//   y  y-coordinate of point
//   z  z-coordinate of point
//   r  outer-most radius of dot {6}
//   selected  {false}
inline void draw_point(
  double x,
  double y,
  double z,
  double r = 7,
  bool selected = false);

// Implementation
#if __APPLE__
#  include <OpenGL/gl.h>
#else
#  ifdef _WIN32
#    define NOMINMAX
#    include <Windows.h>
#    undef NOMINMAX
#  endif
#  include <GL/gl.h>
#endif

inline void draw_point(double x, double y, double z,double r, bool selected)
{
  // Push GL settings
  GLboolean old_depth_test;
  glGetBooleanv(GL_DEPTH_TEST,&old_depth_test);
  GLboolean old_lighting;
  glGetBooleanv(GL_LIGHTING,&old_lighting);

  glDisable(GL_DEPTH_TEST);
  glDisable(GL_LIGHTING);

  // get current color
  float color[4];
  glGetFloatv(GL_CURRENT_COLOR,color);

  // White outline
  glColor4f(1,1,1,color[3]);
  glPointSize(2*r);
  glBegin(GL_POINTS);
  glVertex3d(x,y,z);
  glEnd();
  // Black outline
  glColor4f(0,0,0,color[3]);
  glPointSize(2*r-2);
  glBegin(GL_POINTS);
  glVertex3d(x,y,z);
  glEnd();
  // Foreground
  glColor4fv(color);
  glPointSize(2*r-4);
  glBegin(GL_POINTS);
  glVertex3d(x,y,z);
  glEnd();
  // Selection inner circle
  if(selected)
  {
    glColor4f(0,0,0,color[3]);
    glPointSize(2*r-7);
    glBegin(GL_POINTS);
    glVertex3d(x,y,z);
    glEnd();
  }

  // reset color
  glColor4fv(color);

  // Pop GL settings
  if(old_lighting) glEnable(GL_LIGHTING);
  if(old_depth_test) glEnable(GL_DEPTH_TEST);
}


//added by wangyu
inline void draw_point_with_depth(double x, double y, double z,double r=7,bool selected=false, double red=0, double green=0, double blue=0)
{
	// Push GL settings
	//GLboolean old_depth_test;
	//glGetBooleanv(GL_DEPTH_TEST,&old_depth_test);
	GLboolean old_lighting;
	glGetBooleanv(GL_LIGHTING,&old_lighting);

	//glDisable(GL_DEPTH_TEST);
	glDisable(GL_LIGHTING);

	// get current color
	float color[4];
	glGetFloatv(GL_CURRENT_COLOR,color);

	// White outline
	//glColor4f(1,1,1,color[3]);
	//glPointSize(2*r);
	//glBegin(GL_POINTS);
	//glVertex3d(x,y,z);
	//glEnd();
	// Black outline
	//glColor4f(0,0,0,color[3]);
	//glPointSize(2*r-2);
	//glBegin(GL_POINTS);
	//glVertex3d(x,y,z);
	//glEnd();


	// Selection inner circle
	if(selected)
	{
		glColor4f(0,0,1,color[3]);
		//glColor4f(red,green,blue,color[3]);
		glPointSize(2*r);
		glBegin(GL_POINTS);
		glVertex3d(x,y,z);
		glEnd();
	}
	else
	{
		////glColor4fv(color);
		////glColor4f(0,0,0,color[3]);

		glColor4f(red,green,blue,color[3]);
		glPointSize(2*r-4);
		glBegin(GL_POINTS);
		glVertex3d(x,y,z);
		glEnd();

	}

	// reset color
	glColor4fv(color);

	// Pop GL settings
	if(old_lighting) glEnable(GL_LIGHTING);
	//if(old_depth_test) glEnable(GL_DEPTH_TEST);
}
#endif
