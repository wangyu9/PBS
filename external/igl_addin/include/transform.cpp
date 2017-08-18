#include "transform.h"

#ifdef __APPLE__
#   include <OpenGL/gl.h>
#   include <OpenGL/glu.h>
#   include <GLUT/glut.h>
#else
#   ifdef _WIN32
//#       define NOMINMAX // disables min/max marco of WinDef.h
#       include <windows.h>
#       include <GL/glew.h>
#       include <GL/glut.h>
#   endif
#   include <GL/gl.h>
#   include <GL/glu.h>
#endif


#include <Eigen/Geometry>
#include <math_helper.h>

Eigen::Vector3f screen_to_world(float x, float y, float z, double m_modelview_matrix[16], double m_projection_matrix[16], GLint m_viewport[4])
{
	GLfloat winX, winY;
	Eigen::Vector3d point;

	winX = (float)x;
	winY = (float)m_viewport[3] - (float)y;

	gluUnProject(winX, winY, z, m_modelview_matrix, m_projection_matrix, m_viewport, (GLdouble*)&point[0], (GLdouble*)&point[1], (GLdouble*)&point[2]);

	return Eigen::Vector3f(point[0],point[1],point[2]);
}

// calculate a translation given the mouse location and the mouse location when the dragging began
void get_translation(
	int mouse_x,
	int from_x,
	int mouse_y,
	int from_y,
	double depth,
	double *modelview_matrix,
	double *projection_matrix,
	int *viewport,
	float *translation,
	bool *selected_axis  //if this is set, we will only rotate along the axis i for which selected_axis[i] = true
	)
{
	//translation
	Eigen::Vector3f from = screen_to_world(from_x,from_y,depth, 
		modelview_matrix, projection_matrix, viewport);
	Eigen::Vector3f to = screen_to_world(mouse_x,mouse_y,depth,
		modelview_matrix, projection_matrix, viewport);

	translation[0] = to[0] - from[0];
	translation[1] = to[1] - from[1];
	translation[2] = to[2] - from[2];

	//if we are only moving along a selected axis, remove any translation along the rest of the axes
	for (int i = 0 ; i<3; ++i)
		if(selected_axis && !selected_axis[i])
			translation[i] = 0.;

	//printf("Mouse From (%d,%d) to (%d,%d), Translation: (%f,%f,%f)\n",from_x,from_y,mouse_x,mouse_y,translation[0],translation[1],translation[2]);//wangyu
}

void get_translation(
	int mouse_x,
	int from_x,
	int mouse_y,
	int from_y,
	double depth,
	float *translation)
{
	// Put model, projection, and viewport matrices into double arrays
	double MV[16];
	double PM[16];
	int VP[4];
	glGetDoublev(GL_MODELVIEW_MATRIX, MV);
	glGetDoublev(GL_PROJECTION_MATRIX, PM);
	glGetIntegerv(GL_VIEWPORT, VP);

	return get_translation(mouse_x, from_x, mouse_y, from_y, depth, MV, PM, VP, translation);
}


// calculate a rotation given the mouse location and the mouse location when the dragging began
void get_rotation(int mouse_x,
                                 int from_x,
                                 int mouse_y,
                                 int from_y,
                                 double depth,
                                 double *modelview_matrix,
                                 double *projection_matrix,
                                 int *viewport,
                                 float *rotation,
                                 bool *selected_axis //if this is set, we will only rotate along the axis i for which selected_axis[i] = true
                                 )
{
  GLfloat winX, winY;               // Holds Our X, Y Coordinates
  double x,y,z;
  
  winX = (float)mouse_x;                  // Holds The Mouse X Coordinate
  winY = (float)mouse_y;
  winY = (float)viewport[3] - winY;
  gluUnProject(winX, winY, depth, modelview_matrix,projection_matrix,viewport, &x, &y, &z);
  Vector3 p1;
  p1<<x,y,z;
  
  winX = (float)from_x;                  // Holds The Mouse X Coordinate
  winY = (float)from_y;
  winY = (float)viewport[3] - winY;
  gluUnProject(winX, winY, depth, modelview_matrix,projection_matrix,viewport, &x, &y, &z);
  Vector3 p2;
  p2<<x,y,z;
  
  //axis of rotation
  Vector3 axis = p1.cross(p2);
  //if we are only moving around a selected axis, remove any rotation around the rest of the axes
  for (int i = 0 ; i<3; ++i)
    if(selected_axis && !selected_axis[i])
      axis[i] = 0.;
  axis.normalize();
  
  float angle = M_PI*((mouse_x - from_x)*(mouse_x - from_x) + (mouse_y - from_y)*(mouse_y - from_y)) / max(viewport[2],viewport[3]);//wangyu//std::max(viewport[2],viewport[3]) ;
  
  Eigen::Quaternion<ScalarType> rot(Eigen::AngleAxis<ScalarType>(angle,axis));
  
  if (!(rot.x() == rot.x()) ||
      !(rot.y() == rot.y()) ||
      !(rot.z() == rot.z()) ||
      !(rot.w() == rot.w()) )
  {
    /*std::cerr<<"p1: "<<p1.transpose()<<std::endl;
    std::cerr<<"p2: "<<p2.transpose()<<std::endl;
    std::cerr<<"axis: "<<axis.transpose()<<std::endl;
    std::cerr<<"angle: "<<angle<<std::endl;
    std::cerr<<"quaternion: "<<rot.x()<<" "<<rot.y()<<" "<<rot.z()<<" "<<rot.w()<<" "<<std::endl;*/
    rotation[0] = 0.;
    rotation[1] = 0.;
    rotation[2] = 0.;
    rotation[3] = 1.;
    return;
  }
  rotation[0] = rot.x();
  rotation[1] = rot.y();
  rotation[2] = rot.z();
  rotation[3] = rot.w();
}


//implements picking of a face and returns the index of the vertex in the face that was closest to the mouse point
int pick_nearest_handle(const int &mouse_x, // mouse location
	const int &mouse_y,
	double &minsqdist,
	GLint *viewport, //viewport
	const double *projection_matrix, //opengl projection matrix
	const double *modelview_matrix, //opengl modelview matrix
	const std::vector<handleData> &handle_list, //list of vertices
	const int w_x, // optional: width of window around the mouse in which we will search for faces
	const int w_y, // optional: height of window around the mouse in which we will search for faces
	const int *center_x, // optional: center of the window around the mouse in which we will search for faces. If not provided, the mouse position will be used.
	const int *center_y)
{
	int h = handle_list.size();
	Eigen::MatrixXd V(h,3);

	for (int i=0; i<h; i++)
	{
		V(i,0) = handle_list[i].x();
		V(i,1) = handle_list[i].y();
		V(i,2) = handle_list[i].z();
	}

	return pick_nearest_point(
		mouse_x,
		mouse_y,
		minsqdist,
		viewport,
		projection_matrix,
		modelview_matrix,
		V,
		w_x,
		w_y,
		center_x,
		center_y);
}

//implements picking of a face and returns the index of the vertex in the face that was closest to the mouse point
int pick_nearest_point(const int &mouse_x, // mouse location
                                        const int &mouse_y,
										double &minsqdist,
                                        GLint *viewport, //viewport
                                        const double *projection_matrix, //opengl projection matrix
                                        const double *modelview_matrix, //opengl modelview matrix
                                        const Eigen::MatrixXd& V, // points matrix to search in, #V by 3
                                        const int w_x, // optional: width of window around the mouse in which we will search for faces
                                        const int w_y, // optional: height of window around the mouse in which we will search for faces
                                        const int *center_x, // optional: center of the window around the mouse in which we will search for faces. If not provided, the mouse position will be used.
                                        const int *center_y)
{
  long hits = 0;
  GLuint *selectBuf =new GLuint[100000];
  glSelectBuffer(100000, selectBuf);
  glRenderMode(GL_SELECT);
  glInitNames();
  
  
  /* Because LoadName() won't work with no names on the stack */
  glPushName(-1);
  
  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  glLoadIdentity();
  
  /*
   restrict the draw to an area around the cursor
   */
  int cx,cy;
  if(!center_x)
    cx = mouse_x;
  else
    cx = *center_x;
  
  
  if(!center_y)
    cy = viewport[3] - mouse_y;
  else
    cy = viewport[3] - *center_y;
  
  
  //the pick matrix
  gluPickMatrix(cx, cy, w_x, w_y, viewport);
  glMultMatrixd(projection_matrix);
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadIdentity();
  glMultMatrixd(modelview_matrix);
  
  int fcnt=0;
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  

  glPopMatrix();
  glMatrixMode(GL_PROJECTION);
  glPopMatrix();
  glMatrixMode(GL_MODELVIEW);
  hits = glRenderMode(GL_RENDER);
  
  
  size_t picked_vertex = -1;
  //find closest vertex of the face (region under mouse click will be determined from that vertex)
    minsqdist = 1e50;
    for (int i=0; i<V.rows(); i++)
    {
      double x, y, z;
      gluProject(V(i,0),
                 V(i,1),
                 V(i,2),
                 modelview_matrix,
                 projection_matrix,
                 viewport, &x, &y, &z);
      double dist = (x-mouse_x)*(x-mouse_x) + (y-viewport[3]+mouse_y)*(y-viewport[3]+mouse_y);
      if (minsqdist>dist)
      {
        minsqdist = dist;
        picked_vertex = i;
      }
    }

  return picked_vertex;
}


bool ray_trace_mesh(	int & minZ_vertex,
						int & maxZ_vertex,
						const int &mouse_x, // mouse location
                        const int &mouse_y,
                        GLint *viewport, //viewport
                        const double *projection_matrix, //opengl projection matrix
                        const double *modelview_matrix, //opengl modelview matrix
                        const Eigen::MatrixXd *vertices, //list of vertices
                        const Eigen::MatrixXi *faces, //list of faces
                        std::vector< std::pair<double,IndexType> > &hit_face_list, // output: list of hit faces
						const double m_preview_height,
						const int condidate_size,//const double cursor_search_radius,
                        const std::vector<int > &face_indices, // optional: list of face indices within faces that should be tested for picking.
                        const std::vector<int > &face_labels, // optional: labels of faces that should be tested for picking. Should be of the same size as face_indices.
                        const int w_x, // optional: width of window around the mouse in which we will search for faces
                        const int w_y, // optional: height of window around the mouse in which we will search for faces
                        const int *center_x, // optional: center of the window around the mouse in which we will search for faces. If not provided, the mouse position will be used.
                        const int *center_y
						)
{
  long hits = 0;
  GLuint *selectBuf =new GLuint[100000];
  glSelectBuffer(100000, selectBuf);
  glRenderMode(GL_SELECT);
  glInitNames();
  
  if (!(face_indices.empty()))
  {
    if (face_indices.size() != face_labels.size())
      exit(-1);
  }
  
  /* Because LoadName() won't work with no names on the stack */
  glPushName(-1);
  
  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  glLoadIdentity();
  


  GLboolean old_depth_test;
  glGetBooleanv(GL_DEPTH_TEST,&old_depth_test);
  GLboolean old_lighting;
  glGetBooleanv(GL_LIGHTING,&old_lighting);


  /*
   restrict the draw to an area around the cursor
   */
  int cx,cy;
  if(!center_x)
    cx = mouse_x;
  else
    cx = *center_x;
  
  
  if(!center_y)
    cy = viewport[3] - mouse_y;
  else
    cy = viewport[3] - *center_y;
  
  
  //double v[3];
  //v[0] = m_preview->center[0] - m_preview->eye[0];
  //v[1] = m_preview->center[1] - m_preview->eye[1];
  //v[2] = m_preview->center[2] - m_preview->eye[2];
  //Eigen::Vector4d V_world(v[0],v[1],v[2],1);
  //Eigen::Matrix4d ModelViewMatrix(m_preview->m_modelview_matrix);
  //ModelViewMatrix(0,3) = 0;
  //ModelViewMatrix(1,3) = 0;
  //ModelViewMatrix(2,3) = 0;
  //ModelViewMatrix(3,3) = 1;
  //Eigen::Vector4d V_object = ModelViewMatrix.inverse()*V_world;


  //the pick matrix
  gluPickMatrix(cx, cy, w_x, w_y, viewport);
  glMultMatrixd(projection_matrix);
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadIdentity();
  glMultMatrixd(modelview_matrix);
  
  int fcnt=0;
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  
  ////fake-draw the faces
  //if (face_indices.empty())
  //{
  //  for(int fi=0;fi<faces->rows();++fi)
  //  {
  //    glLoadName(fcnt);
  //    glBegin(GL_POLYGON);
  //    for(int vit = 0; vit < faces->cols(); ++vit)
  //    {
  //      const ScalarType *vertex_data = vertices->data() + 3* (*faces)(fi,vit);
  //      glVertex3f(vertex_data[0], vertex_data[1], vertex_data[2]);
  //    }
  //    glEnd();
  //    fcnt++;
  //  }
  //}
  //else
  //{
  //  for(unsigned long fi=0;fi<face_indices.size();++fi)
  //  {
  //    glLoadName(face_labels[fi]);
  //    glBegin(GL_POLYGON);
  //    for(int vit = 0; vit < faces->cols(); ++vit)
  //    {
  //      const ScalarType *vertex_data = vertices->data() + 3* (*faces)(face_indices[fi],vit);
  //      glVertex3f(vertex_data[0], vertex_data[1], vertex_data[2]);
  //    }
  //    glEnd();
  //    fcnt++;
  //  }
  //}
  glPopMatrix();
  glMatrixMode(GL_PROJECTION);
  glPopMatrix();
  glMatrixMode(GL_MODELVIEW);
  hits = glRenderMode(GL_RENDER);
  
  hit_face_list.clear();
  GLuint *ptr = (GLuint *) selectBuf;
  for(long ii=0;ii<hits;ii++)
  {
    int numNames = *ptr++;
    float minZ = (float) *ptr++/0x7fffffff;
    hit_face_list.push_back( std::pair<double,IndexType>(minZ,*(++ptr)));
    ptr+=numNames;
  }
  std::sort(hit_face_list.begin(),hit_face_list.end());
  delete [] selectBuf;
  

  printf("Hit face list size:%d\n",hit_face_list.size());
  minZ_vertex = -1;
  maxZ_vertex = -1;
  //find closest vertex of the face (region under mouse click will be determined from that vertex)
  //const double sqdist_threshold = cursor_search_radius*cursor_search_radius;


  std::vector< std::tuple<double, int, double> > closet_sq_dist;

  if(true)//hits)
  {
	for(long i = 0; i<vertices->rows(); i++)
	{
		{
		  double x, y, z;

		  gluProject((*vertices)(i,0),
					(*vertices)(i,1),
					(*vertices)(i,2),
			  modelview_matrix,
			  projection_matrix,
			  viewport, &x, &y, &z);
		  y = m_preview_height - y;
		  double sqdist_2d_to_mouse 
			  = (x-mouse_x)*(x-mouse_x)
			  + (y-mouse_y)*(y-mouse_y);
		  //if(sqdist_2d_to_mouse>sqdist_threshold)
  
		  double rank = sqrt(sqdist_2d_to_mouse);// +z;// used for ranking

		  if (closet_sq_dist.size() >= condidate_size)  
		  {
			  double d0;
			  int nusedi; double nusedz;// not ever used, only for tie()
			  std::tie(d0, nusedi, nusedz) = closet_sq_dist.back();
			  if (rank > d0)
			  {
				  continue;
			  }	  
		  }

		  if (closet_sq_dist.size() >= condidate_size)
		  {
			  closet_sq_dist.back() = std::make_tuple(rank, i, z);
		  }
		  else
		  {
			  closet_sq_dist.push_back(std::make_tuple(rank, i, z));
		  }
		  

		  // bubble sort for the small set:
		  for (int i = closet_sq_dist.size() - 2; i>=0; i--)
		  {
			  double di;
			  double dip1;
			  int nusedi; double nusedz;// not ever used, only for tie()
			  std::tie(di, nusedi, nusedz) = closet_sq_dist[i];
			  std::tie(dip1, nusedi, nusedi) = closet_sq_dist[i + 1];
			  if (di>dip1)
			  {
				  closet_sq_dist[i].swap(closet_sq_dist[i+1]);
			  }
		  }
		}
	}

	double minZ_rank = 1e50;
	//double maxZ = -1e50;
	for (int i = 0; i < closet_sq_dist.size(); i++)
	{
		double sqdist;
		int index; 
		double rank;
		std::tie(sqdist,index,rank) = closet_sq_dist[i];
		if (minZ_rank > rank)
		{
			hits = true;
			minZ_rank = rank;
			minZ_vertex = index;
			maxZ_vertex = index;// this is not implemented yet!
		}
		//if (maxZ < z)
		//{
		//	hits = true;
		//	maxZ = z;
		//	maxZ_vertex = index;
		//}
	}
  }

  // Pop GL settings
  if(old_lighting) glEnable(GL_LIGHTING);
  if(old_depth_test) glEnable(GL_DEPTH_TEST);

  return hits;
}



void ConvertQuaternionToMatrix4x4(const float *quat, Eigen::MatrixXd& Mat)
{
	float mat[4*4];
	ConvertQuaternionToMatrix(quat, mat);
	Mat.resize(4,4);
	for(int i=0; i<4; i++)
	{
		for(int j=0; j<4; j++)
		{
			Mat(i,j) = mat[4*i+j];
		}
	}
}

void ConvertQuaternionToMatrix3x3(const float *quat, Eigen::MatrixXd& Mat)
{
	float mat[4*4];
	ConvertQuaternionToMatrix(quat, mat);
	Mat.resize(3,3);
	for(int i=0; i<3; i++)
	{
		for(int j=0; j<3; j++)
		{
			Mat(i,j) = mat[4*i+j];
		}
	}
}

void rotate_around_axis(const double v0[3], const double center[3], const double axis[3], const double angle, double v1[3])
{
	float quat[4];
	quat[0] = axis[0]*sin(angle);
	quat[1] = axis[1]*sin(angle);
	quat[2] = axis[2]*sin(angle);
	quat[3] = cos(angle);
	Eigen::Vector4d p(v0[0]-center[0],v0[1]-center[1],v0[2]-center[2],1);
	float mat[4*4];
	ConvertQuaternionToMatrix(quat, mat);
	Eigen::MatrixXd rot_mat(4,4);
	for(int i=0; i<4; i++)
	{
		for(int j=0; j<4; j++)
		{
			rot_mat(i,j) = mat[4*i+j];
		}
	}
	Eigen::Vector4d rot_p = rot_mat * p;
	rot_p += Eigen::Vector4d(center[0],center[1],center[2],0);
	v1[0] = rot_p[0];
	v1[1] = rot_p[1];
	v1[2] = rot_p[2];
}
//
//void rotate_selected_handles_around_axis(const double axis[3], const bool around_center, const double cursor_rotation_center[3], const double rotation_2D, Eigen::MatrixXd rest_Handles, std::vector<handleData>& all_handle_list, std::vector<int>& select_handle_list)
//{
//	int N = select_handle_list.size();
//	if(N<=0)
//	{
//		return;
//	}
//	double avg_x = 0;
//	double avg_y = 0;
//	double avg_z = 0;
//
//	if(true)
//	{
//		avg_x = cursor_rotation_center[0];
//		avg_y = cursor_rotation_center[1];
//		avg_z = cursor_rotation_center[2];
//	}
//
//
//	for(int i=0; i<select_handle_list.size(); i++)
//	{
//		double v0[3];
//		v0[0] = rest_Handles(select_handle_list[i],0);//all_handle_list[select_handle_list[i]].x;
//		v0[1] = rest_Handles(select_handle_list[i],1);//all_handle_list[select_handle_list[i]].y;
//		v0[2] = rest_Handles(select_handle_list[i],2);//all_handle_list[select_handle_list[i]].z;
//
//		double center[3];
//		center[0] = avg_x;
//		center[1] = avg_y;
//		center[2] = avg_z;
//		double v1[3] = {0,0,0};
//		rotate_around_axis(v0,center,axis,rotation_2D,v1);
//		all_handle_list[select_handle_list[i]].x = v1[0];
//		all_handle_list[select_handle_list[i]].y = v1[1];
//		all_handle_list[select_handle_list[i]].z = v1[2];
//	}
//
//}
//
