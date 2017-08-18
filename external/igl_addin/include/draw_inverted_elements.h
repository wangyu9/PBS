#ifndef IGL_ADDIN_DRAW_INVERTED_ELEMENTS
#define IGL_ADDIN_DRAW_INVERTED_ELEMENTS

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

#include <igl/igl_inline.h>
#include <Eigen/Core>

template <typename DerivedV, typename DerivedTF>
void IGL_INLINE draw_inverted_elements(const Eigen::PlainObjectBase<DerivedV> & V, const Eigen::PlainObjectBase<DerivedTF> & TF)
{
	printf("Not Finished yet, need to change Eigne Usage!\n");

	// Render flipped tets
	glDisable(GL_LIGHTING);
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_CULL_FACE);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	glColor3f(1,1,0);
	if(TF.cols()==4)
	{
		for(int t=0;t<TF.rows();t++)
		{
			Vector4i index = TF.row(t);

			Vector3 A = V.row(index[0]);
			Vector3 B = V.row(index[1]);
			Vector3 C = V.row(index[2]);
			Vector3 D = V.row(index[3]);

			Vector3 a = A-D;
			Vector3 b = B-D;
			Vector3 c = C-D;

			double det = a.dot(c.cross(b));

			if(det < 0)
			{
				glBegin(GL_TRIANGLES);
				glVertex(A);
				glVertex(B);
				glVertex(C);
				glEnd();

				glBegin(GL_TRIANGLES);
				glVertex(B);
				glVertex(C);
				glVertex(D);
				glEnd();

				glBegin(GL_TRIANGLES);
				glVertex(A);
				glVertex(C);
				glVertex(D);
				glEnd();

				glBegin(GL_TRIANGLES);
				glVertex(A);
				glVertex(B);
				glVertex(D);
				glEnd();
			}
		}
	}
	else
	{
		// Draw flipped triangles
		for(int t=0;t<TF.rows();t++)
		{
			Eigen::Vector3i index = TF.row(t);

			Vector2 A = V.row(index[0]).block<1,2>(0,0);;
			Vector2 B = V.row(index[1]).block<1,2>(0,0);;
			Vector2 C = V.row(index[2]).block<1,2>(0,0);;

			Matrix2d J;
			J.row(0) = A-C;
			J.row(1) = B-C;

			double det = J.determinant();

			if(det < 0)
			{
				glBegin(GL_TRIANGLES);
				glVertex(A);
				glVertex(B);
				glVertex(C);
				glEnd();
			}
		}
	}
}

#endif
 