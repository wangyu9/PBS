#ifndef GL_HELPER_H
#define GL_HELPER_H

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

inline void push_settings(
	const float mat[16],
	const float scale[3],
	const float trans[3],
	const bool enable_depth)
{
	glPushMatrix();

	glMultMatrixf(mat);
	glScaled(scale[0], scale[1], scale[2]);
	glTranslatef(trans[0], trans[1], trans[2]);

}

inline void pop_settings(const bool enable_depth)
{
	glPopMatrix();
}

#endif

