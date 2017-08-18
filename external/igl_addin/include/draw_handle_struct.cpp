#include "draw_handle_struct.h"

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

#include "draw_point.h"
#include <GL_helper.h>

inline void quadricDrawPoint(double x, double y, double z, double diameter, float color[4])
{
	// Push GL Settings
	GLboolean old_lighting;
	glGetBooleanv(GL_LIGHTING,&old_lighting);
	glDisable(GL_LIGHTING);
	// get current color
	float old_color[4];
	glGetFloatv(GL_CURRENT_COLOR,old_color);

	glPushMatrix();

	GLUquadricObj *quadric;
	quadric = gluNewQuadric();
	gluQuadricDrawStyle(quadric, GLU_FILL );

	glTranslatef(x,y,z);
	glColor4f(color[0],color[1],color[2],color[3]);
	gluSphere(quadric,diameter,7,3);

	gluDeleteQuadric(quadric);

	glPopMatrix();

	// Pop GL settings
	glColor4fv(old_color);
	if(old_lighting) 
		glEnable(GL_LIGHTING);

	return;
}

/*
float mat[4 * 4];
ConvertQuaternionToMatrix(m_preview->camera.g_Rotation, mat);
float scale[3];
scale[0] = scale[1] = scale[2] = m_preview->camera.g_Zoom * m_preview->camera.zoom;
float trans[3];
trans[0] = m_preview->camera.g_Translation[0];
trans[1] = m_preview->camera.g_Translation[1];
trans[2] = m_preview->camera.g_Translation[2];
*/


void draw_point_handles(
	const float handle_radius,
	const float handle_color[4],
	const float diameter,
	const float ratio_of_passive,
	const bool draw_region_handles,
	const bool draw_only_point_handle_center,
	const HandleStructure& hs,
	std::vector<bool> bToDraw)
{
	const std::vector<handleData>& all_handle_list = hs.all_handle_list;


	if (bToDraw.size() != all_handle_list.size())
	{
		bToDraw.resize(all_handle_list.size());
		for (int i = 0; i < bToDraw.size(); i++)
		{
			bToDraw[i] = true;// by default draw all handles
		}
	}

	//mainly draw the handles here:
	//assert(Handles.rows()==all_handle_list.size());
	for(int i=0; i<all_handle_list.size(); i++)
	{
		if (!bToDraw[i])
			continue;

		if(true)//(enable_depth)//if(handleDispType==WithDepth)
		{

			if(true)
			{
				//adaptive handle size
#define Adaptive_Handle_size
#ifdef Adaptive_Handle_size
				//double diameter = std::min(m_preview->diameter*0.004,m_preview->avg_edge_length*0.3);
#endif


				float color[4];
				color[0] = handle_color[0];
				color[1] = handle_color[1];
				color[2] = handle_color[2];
				color[3] = handle_color[3];

				if(all_handle_list[i].active)
				{
					color[0] = 255;
					color[1] = 0;
					color[2] = 0;
					color[3] = 0;
				}
				if(all_handle_list[i].selected&&all_handle_list[i].active)
				{
					color[0] = 0;
					color[1] = 0;
					color[2] = 255;
					color[3] = 0;
				}

				const Eigen::MatrixXd& P = all_handle_list[i].Pos();
				if (P.rows()==1)
				{
					quadricDrawPoint(
						P(0,0),
						P(0,1),
						P(0,2), 
						ratio_of_passive*diameter*handle_radius,
						color);
				}
				else
				{
					if (draw_region_handles)
					{
						for (int k = 0; k < P.rows(); k++)
						{
							quadricDrawPoint(
								P(k, 0),
								P(k, 1),
								P(k, 2),
								ratio_of_passive*diameter*handle_radius,
								color);
						}
					}
				}

				const Eigen::MatrixXd& PC = all_handle_list[i].CenterPos();
				if(all_handle_list[i].active)
				{
					if (draw_only_point_handle_center)// only draw point handle
					{
						if (all_handle_list[i].type() != HandleTypePoint) 
							continue;
					}

					// always draw active handles without depth

					// Push GL settings
					GLboolean old_depth_test2;
					glGetBooleanv(GL_DEPTH_TEST, &old_depth_test2);
					glDisable(GL_DEPTH_TEST);

					draw_point(
						PC(0, 0),
						PC(0, 1),
						PC(0, 2),
						handle_radius,
						all_handle_list[i].selected//,
						//.8,.8,.2// yellow for region
						);

					// Pop GL settings
					if (old_depth_test2)
						glEnable(GL_DEPTH_TEST);
					else
						glDisable(GL_DEPTH_TEST);
				}


			}
			else
			{// old // To remove

				const Eigen::MatrixXd& P = all_handle_list[i].Pos();
				for (int k=0; k<P.rows(); k++)
				{
					draw_point_with_depth(
						P(k,0),
						P(k,1),
						P(k,2), 
						handle_radius,
						all_handle_list[i].selected,
						handle_color[0],handle_color[1],handle_color[2]);
				}
			}

		}
		else
		{

			const Eigen::MatrixXd& P = all_handle_list[i].CenterPos();
			if(all_handle_list[i].selected&&all_handle_list[i].active)
			{
				draw_point(
					P(0,0),
					P(0,1),
					P(0,2), 
					handle_radius,
					all_handle_list[i].selected);
			}
		}
		//draw_point(
		//	(double) PickingPlugin::GetReference().all_handle_list,
		//	);
	}

	//for(int i=0; i<HandleFaces.rows(); i++)
	//{
	//	int v1 = HandleFaces(i,0);
	//	int v2 = HandleFaces(i,1);
	//	int	v3 = HandleFaces(i,2);

	//	draw_directed_line_segment2(Handles(v1,0),Handles(v1,1),Handles(v1,2),
	//								Handles(v2,0),Handles(v2,1),Handles(v2,2),3,0,0,0);
	//	draw_directed_line_segment2(Handles(v2,0),Handles(v2,1),Handles(v2,2),
	//								Handles(v3,0),Handles(v3,1),Handles(v3,2),3,0,0,0);
	//	draw_directed_line_segment2(Handles(v3,0),Handles(v3,1),Handles(v3,2),
	//								Handles(v1,0),Handles(v1,1),Handles(v1,2),3,0,0,0);
	//}

}