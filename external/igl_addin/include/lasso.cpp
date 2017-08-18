#define NOMINMAX
// to eliminate the bug of redefine of max min
#include "lasso.h"

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


#include "transform.h"
#include "Viewer.h"

#ifdef USING_IGL_HEADER_ONLY_MODE
#define IGL_HEADER_ONLY 
#endif

#include "picking_util.h"

inline void DrawLassoDashedLine(const std::vector<std::pair<int, int>>& mousePoints)
{
	if (mousePoints.size())
	{
		GLint viewport[4];
		glGetIntegerv(GL_VIEWPORT,viewport);

		glMatrixMode (GL_PROJECTION);
		glPushMatrix();
		glLoadIdentity ();
		gluOrtho2D ((GLdouble) viewport[0], (GLdouble) viewport[2], (GLdouble) viewport[1], (GLdouble) viewport[3]);
		glMatrixMode (GL_MODELVIEW);
		glPushMatrix();
		glLoadIdentity ();

		glDisable(GL_DEPTH_TEST);
		glDisable(GL_CULL_FACE);
		glDisable(GL_BLEND);
		glDisable(GL_TEXTURE_2D);
		glDisable(GL_LIGHTING);
		glDisable (GL_LIGHT0);

		float linewidth;
		glGetFloatv(GL_LINE_WIDTH,&linewidth);
		glLineWidth(2.);
		glLineStipple(2, 0xAAAA);
		glEnable(GL_LINE_STIPPLE);
		glColor3d(0.5, 0.5, 0.5);
		glBegin(GL_LINE_STRIP);
		for (unsigned long i = 0; i <mousePoints.size(); ++i)
			glVertex2d(mousePoints[i].first,viewport[3] - mousePoints[i].second);
		glEnd();
		glDisable(GL_LINE_STIPPLE);
		glLineWidth(linewidth);

		//        glutSwapBuffers();

		glEnable(GL_DEPTH_TEST);

		glMatrixMode (GL_MODELVIEW);
		glPopMatrix();
		glMatrixMode (GL_PROJECTION);
		glPopMatrix();
		glMatrixMode (GL_MODELVIEW);
	}
}

void Lasso::draw()
{
	//if in the process of selection, draw a dashed line with the lasso (2-d drawing)
	if (selecting)
	{
		DrawLassoDashedLine(mousePoints);
	}
}

bool Lasso::is_activate()
{
	return activate;
}

bool Lasso::mouseMove(int mouse_x, int mouse_y, bool& calledLassoMove, bool move_in_plane)
{
	// Put model, projection, and viewport matrices into double arrays
	double MV[16];
	double PM[16];
	int VP[4];
	glGetDoublev(GL_MODELVIEW_MATRIX, MV);
	glGetDoublev(GL_PROJECTION_MATRIX, PM);
	glGetIntegerv(GL_VIEWPORT, VP);

	return mouseMove(mouse_x, mouse_y, MV, PM, VP, calledLassoMove, move_in_plane);
}

bool Lasso::mouseMove(
	int mouse_x, 
	int mouse_y, 
	//void (*pLassoMove)(),//void(_cdecl HandlePlugin::*)(void) pLassoMove,//void (*pLassoMove)(), 	
	double *modelview_matrix,
	double *projection_matrix,
	int *viewport,
	bool& calledLassoMove,
	bool move_in_plane)
{
	calledLassoMove = false;

	if (selecting)//if in selection mode: lasso tool (add mouse location to the lasso polyline)
	{
		//keep tracking the mouse: gather up the points
		mousePoints.push_back(std::make_pair(mouse_x, mouse_y));
		return true;
	}
	//switch (mouse_mode)
	{
		if (!activate )//|| m_currentRegion <0)
			return false;//break;

		float tp[3];// this translation is always parallel to the screen.

		get_translation(mouse_x,
			from_x,
			mouse_y,
			from_y,
			depth_at_mouse_pos,
			modelview_matrix,//m_preview->m_modelview_matrix,
			projection_matrix,//m_preview->m_projection_matrix,
			viewport,//m_preview->m_viewport,
			tp,
			0//(m_USE_DISPLAYED_AXES)?axis_is_selected:0
			);

		if (move_in_plane)
		{

			//translation
			Eigen::Vector3f from = screen_to_world(mouse_x, mouse_y, 0,
				modelview_matrix, projection_matrix, viewport);
			Eigen::Vector3f to = screen_to_world(mouse_x, mouse_y, -1,
				modelview_matrix, projection_matrix, viewport);

			Eigen::Vector3f p = to - from; // this is the vector that is pendicular to the screen;
			p.normalize();
			Eigen::Vector3f n;// this is the normal direction of the plane that the points should stay on.
			n << 1, 0, 0;
			n.normalize();

			float t = 0;
			
			if ((n(0)*p(0) + n(1)*p(1) + n(2)*p(2))<0.0001)
			{
				printf("Warning: plane is parallel to the mouse translation, no project!\n");
			} 
			else
			{
				t = -(n(0)*tp[0] + n(1)*tp[1] + n(2)*tp[2]) / (n(0)*p(0) + n(1)*p(1) + n(2)*p(2));
			}
			
			translation[0] = tp[0] + t * p(0);
			translation[1] = tp[1] + t * p(1);
			translation[2] = tp[2] + t * p(2);

			std::cout << "vector p:\n" << std::endl;
			std::cout << "vector n:\n" << std::endl;
			std::cout << "point tp:\n" << tp[0] << tp[1] << tp[2] << std::endl;
			std::cout << "t:\n" << t << std::endl;
		} 
		else
		{
			translation[0] = tp[0];
			translation[1] = tp[1];
			translation[2] = tp[2];
		}

		if(m_UPDATE_WHILE_MOVING)
		{
			//update the constrained vertices (selected regions and handle), calculate the deformation and update the vertices
			from_x = mouse_x;
			from_y = mouse_y;

			//(*pLassoMove)();
			calledLassoMove = true;
		}
		return true;
	}
}

bool Lasso::mouseDown(int mouse_x, int mouse_y, int modifiers, const Eigen::MatrixXd& P, bool& calledLassoDown, int& selected_index)
{
	// Put model, projection, and viewport matrices into double arrays
	double MV[16];
	double PM[16];
	int VP[4];
	glGetDoublev(GL_MODELVIEW_MATRIX, MV);
	glGetDoublev(GL_PROJECTION_MATRIX, PM);
	glGetIntegerv(GL_VIEWPORT, VP);

	return mouseDown(mouse_x, mouse_y, MV, PM, VP, modifiers, P, calledLassoDown, selected_index);
}

bool Lasso::mouseDown(
	int mouse_x, 
	int mouse_y,
	double *modelview_matrix,
	double *projection_matrix,
	int *viewport,
	int modifiers,
	//const std::vector<handleData>& all_handle_list,
	const Eigen::MatrixXd& P,
	// output:
	bool& calledLassoDown,
	int& selected_index
	)
{

	calledLassoDown = false;
	selected_index = -1;

	// save state before translation/rotation
	from_x = mouse_x;
	from_y = mouse_y;

	// No use for HandlePlugin
	//previous_m_cpoint3 << m_cpoint3;
	//*previous_vertices << *import_vertices;

	//glReadPixels(mouse_x, m_preview->m_viewport[3] - mouse_y, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &depth_at_mouse_pos);
	glReadPixels(mouse_x, viewport[3] - mouse_y, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &depth_at_mouse_pos);


	// No use for HandlePlugin
	//m_currentRegion = -1;

	activate = false;

	if(modifiers == Preview3D::ALT)
	{
		//lassoDownAlt(mouse_x,mouse_y);
		calledLassoDown = true;
		return true;
	}

	if(modifiers ==  Preview3D::SHIFT)
		// selection/undo selection mode : start tracking mouse path for deselecting enclosed vertices (lasso tool)
	{
		//lassoDownShift();
		calledLassoDown = true;

		mousePoints.clear();
		//insert the first point
		mousePoints.push_back(std::make_pair(mouse_x, mouse_y));
		selecting = true;
		return true;
	}

	if(modifiers == Preview3D::CTRL)
	{
		return true;
	}
	else
	{// None modifier
		double minsqdist;
		selected_index = pick_nearest_point(mouse_x,
			mouse_y,
			minsqdist,
			viewport,//m_preview->m_viewport,
			projection_matrix,//m_preview->m_projection_matrix,
			modelview_matrix,//m_preview->m_modelview_matrix,
			P//all_handle_list
			);
		if(minsqdist>200)
			selected_index = -1;

		if(selected_index>=0)
		{
			calledLassoDown = true;
			//lassoDownNone(mouse_x,mouse_y,selected_index);

			activate = true;
			return true;
		}
		else
		{
			//printf("No Handle is selected!\n");
			return false;
		}
	}
}

bool Lasso::mouseUp(int mouse_x, int mouse_y, const Eigen::MatrixXd& P, bool& calledLassoUp, bool& calledLassoMove, std::vector<int>& indics_inside_lasso)
{
	// Put model, projection, and viewport matrices into double arrays
	double MV[16];
	double PM[16];
	int VP[4];
	glGetDoublev(GL_MODELVIEW_MATRIX, MV);
	glGetDoublev(GL_PROJECTION_MATRIX, PM);
	glGetIntegerv(GL_VIEWPORT, VP);

	return mouseUp(mouse_x, mouse_y, MV, PM, VP, P, calledLassoUp, calledLassoMove, indics_inside_lasso);
}

bool Lasso::mouseUp(
	int mouse_x, 
	int mouse_y,
	double *modelview_matrix,
	double *projection_matrix,
	int *viewport,
	//const std::vector<handleData>& all_handle_list,
	const Eigen::MatrixXd& P,
	// output:
	bool& calledLassoUp,
	bool& calledLassoMove,
	std::vector<int>& indics_inside_lasso
	)
{
	calledLassoMove = calledLassoUp = false;
	indics_inside_lasso.clear();
	/********************Start of lasso tool**********************/ 

	if(selecting) // if we are in selection mode: gather up any faces inside the bounding box of the lasso, and then check separately which of the vertices are inside the lasso polyline
	{
		//std::vector<int> indics_inside_lasso;

		if(true)
		{//for Handle_tranlate and Handle_rotate
			//for selection and deselection: find vertices inside the polyline the user has drawn. Then mark/unmark them.
			//step 1: use the bounding box of the polyline and OpenGL picking to isolate potentially selected faces
			int minx = 1e5, maxx = -1, miny = 1e5, maxy = -1;
			for (unsigned long i = 0; i<mousePoints.size(); ++i)
			{
				minx = std::min(minx,mousePoints[i].first);
				maxx = std::max((float)maxx,(float)mousePoints[i].first);
				miny = std::min(miny,mousePoints[i].second);
				maxy = std::max(maxy,mousePoints[i].second);
			}
			int width = maxx - minx;
			int height = maxy - miny;
			int cx = minx + 0.5*width;
			int cy = miny + 0.5*height;

			/********** Inner Function: Selection ***********/

			double minsqdist;
			int selected_index = pick_nearest_point(mouse_x,
				mouse_y,
				minsqdist,
				viewport,//m_preview->m_viewport,
				projection_matrix,//m_preview->m_projection_matrix,
				modelview_matrix,//m_preview->m_modelview_matrix,
				P//all_handle_list
				);

			/************************************************/

			//step2: check all vertices of the selected faces and see if they fall inside the polyline or not
			std::vector<std::vector<IndexType > >poly;
			poly.resize(mousePoints.size());
			for(unsigned long i=0; i<mousePoints.size(); ++i)
			{
				poly[i].resize(2);
				poly[i][0] = mousePoints[i].first;
				poly[i][1] = viewport[3] - mousePoints[i].second;//m_preview->m_viewport[3] - mousePoints[i].second;
			}

			//for(int i=0; i<all_handle_list.size(); i++)
			for(int i=0; i<P.rows(); i++)
			{
				double x, y, z;
				gluProject(
					P(i,0),//all_handle_list[i].x(),
					P(i,1),//all_handle_list[i].y(),
					P(i,2),//all_handle_list[i].z(),
					modelview_matrix,//m_preview->m_modelview_matrix,
					projection_matrix,//m_preview->m_projection_matrix,
					viewport,//m_preview->m_viewport, 
					&x, &y, &z
					);

				if (inpoly(poly, x, y))
				{
					indics_inside_lasso.push_back(i);
				}
			}
		}

		/************ Response Function Area *************/

		calledLassoUp = true;
		calledLassoMove = true;
		//lassoUp(indics_inside_lasso);

		/************************************************/

		mousePoints.clear();
		//colorSelection();
		selecting = false;

		translation[0] = 0.;
		translation[1] = 0.;
		translation[2] = 0.;

		return true;
	}
	else
	{
		translation[0] = 0.;
		translation[1] = 0.;
		translation[2] = 0.;

		if (activate)
		{
			activate = false;
			return true;
		}
		else
		{
			return false;
		}
	}
}

Lasso::Lasso()
{
	// Init lasso tools
	mousePoints.clear();
	translation[0] = 0.;
	translation[1] = 0.;
	translation[2] = 0.;
	depth_at_mouse_pos = 0.;
	//rotation[0] = 0.;
	//rotation[1] = 0.;
	//rotation[2] = 0.;
	//rotation[3] = 1.;

	m_UPDATE_WHILE_MOVING = true;

	selecting = false;

	activate = false;

	from_x = from_y = 0;
	to_x = to_y = 0;
}