#include "picking_util.h"

#include "draw_primitives.h"
#include "draw_frame.h"

#include <queue>
#include <algorithm>
#include <set>

#define SELECT_BUFFER_SIZE 1000000
// Default value is 100000

//implements picking of one of the coordinate axes
int pick_axes(const Vector3 &point, //3d point corresponding to the center of the coordinate frame
	const double length, //length of the arrows
	const double radius, //radius of the arrows
	const int &mouse_x, //mouse location
	const int &mouse_y, //mouse location
	GLint *viewport, //opengl viewport
	const double *projection_matrix, //opengl projection matrix
	const double *modelview_matrix, //opengl modelview matrix
	const int w_x,  // optional: width of window around the mouse in which we will look for the selected axes
	const int w_y  // optional: height of window around the mouse in which we will search for the selected axes
	)
{
	long hits = 0;
	GLuint *selectBuf = new GLuint[SELECT_BUFFER_SIZE];
	glSelectBuffer(SELECT_BUFFER_SIZE, selectBuf);
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
	int cx = mouse_x;
	int cy = viewport[3] - mouse_y;

  gluPickMatrix(cx, cy, w_x, w_y, viewport);
  glMultMatrixd(projection_matrix);
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadIdentity();
  glMultMatrixd(modelview_matrix);
  
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  Matrix33 I = Matrix33::Identity();
  //fake-draw the axes
  for (int i =0; i <3; ++i)
  {
    glLoadName(i);
    paintArrow(point, point + length*I.col(i), radius);
  }
  glPopMatrix();
  glMatrixMode(GL_PROJECTION);
  glPopMatrix();
  glMatrixMode(GL_MODELVIEW);
  hits = glRenderMode(GL_RENDER);
  
  std::vector< std::pair<double,unsigned int> > hit_face_list;
  hit_face_list.clear();
  GLuint *ptr = (GLuint *) selectBuf;
  for(long ii=0;ii<hits;ii++)
  {
    int numNames = *ptr++;
    float minZ = (float) *ptr++/0x7fffffff;
    hit_face_list.push_back( std::pair<double,unsigned int>(minZ,*(++ptr)));
    ptr+=numNames;
  }
  std::sort(hit_face_list.begin(),hit_face_list.end());
  delete [] selectBuf;
  
  int picked_axis = -1;
  if(hits)
    picked_axis = hit_face_list[0].second;
  
  return picked_axis;
}




//implements picking of a face and returns the index of the vertex in the face that was closest to the mouse point
int pick_face_and_vertex(const int &mouse_x, // mouse location
                                        const int &mouse_y,
                                        GLint *viewport, //viewport
                                        const double *projection_matrix, //opengl projection matrix
                                        const double *modelview_matrix, //opengl modelview matrix
                                        const Eigen::MatrixXd *vertices, //list of vertices
                                        const Eigen::MatrixXi *faces, //list of faces
                                        std::vector< std::pair<double,IndexType> > &hit_face_list, // output: list of hit faces
                                        const std::vector<int > &face_indices, // optional: list of face indices within faces that should be tested for picking.
                                        const std::vector<int > &face_labels, // optional: labels of faces that should be tested for picking. Should be of the same size as face_indices.
                                        const int w_x, // optional: width of window around the mouse in which we will search for faces
                                        const int w_y, // optional: height of window around the mouse in which we will search for faces
                                        const int *center_x, // optional: center of the window around the mouse in which we will search for faces. If not provided, the mouse position will be used.
                                        const int *center_y)
{
  long hits = 0;
  GLuint *selectBuf = new GLuint[SELECT_BUFFER_SIZE];
  glSelectBuffer(SELECT_BUFFER_SIZE, selectBuf);
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
  
  //fake-draw the faces
  if (face_indices.empty())
  {
    for(int fi=0;fi<faces->rows();++fi)
    {
      glLoadName(fcnt);
      glBegin(GL_POLYGON);
      for(int vit = 0; vit < faces->cols(); ++vit)
      {
        const ScalarType *vertex_data = vertices->data() + 3* (*faces)(fi,vit);
        glVertex3f(vertex_data[0], vertex_data[1], vertex_data[2]);
      }
      glEnd();
      fcnt++;
    }
  }
  else
  {
    for(unsigned long fi=0;fi<face_indices.size();++fi)
    {
      glLoadName(face_labels[fi]);
      glBegin(GL_POLYGON);
      for(int vit = 0; vit < faces->cols(); ++vit)
      {
        const ScalarType *vertex_data = vertices->data() + 3* (*faces)(face_indices[fi],vit);
        glVertex3f(vertex_data[0], vertex_data[1], vertex_data[2]);
      }
      glEnd();
      fcnt++;
    }
  }
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
  
  size_t picked_vertex = -1;
  //find closest vertex of the face (region under mouse click will be determined from that vertex)
  if(hits)
  {
    long fi = hit_face_list[0].second;
    double mindist = 1e50;
    for (int vi = 0; vi<faces->cols(); ++vi)
    {
      double x, y, z;
      gluProject((*vertices)((*faces)(fi,vi),0),
                 (*vertices)((*faces)(fi,vi),1),
                 (*vertices)((*faces)(fi,vi),2),
                 modelview_matrix,
                 projection_matrix,
                 viewport, &x, &y, &z);
      double dist = (x-mouse_x)*(x-mouse_x) + (y-viewport[3]+mouse_y)*(y-viewport[3]+mouse_y);
      if (mindist>dist)
      {
        mindist = dist;
        picked_vertex = (*faces)(fi,vi);
      }
    }
    
  }
  return picked_vertex;
}

//implements picking of a face and returns the index of the vertex in the face that was closest to the mouse point
int pick_nearest_vertex(const int &mouse_x, // mouse location
                                        const int &mouse_y,
										double &minsqdist,
                                        GLint *viewport, //viewport
                                        const double *projection_matrix, //opengl projection matrix
                                        const double *modelview_matrix, //opengl modelview matrix
                                        const Eigen::MatrixXd *vertices, //list of vertices
                                        const int w_x, // optional: width of window around the mouse in which we will search for faces
                                        const int w_y, // optional: height of window around the mouse in which we will search for faces
                                        const int *center_x, // optional: center of the window around the mouse in which we will search for faces. If not provided, the mouse position will be used.
                                        const int *center_y)
{
  long hits = 0;
  GLuint *selectBuf = new GLuint[SELECT_BUFFER_SIZE];
  glSelectBuffer(SELECT_BUFFER_SIZE, selectBuf);
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
    for (int i=0; i<vertices->rows(); i++)
    {
      double x, y, z;
      gluProject((*vertices)(i,0),
                 (*vertices)(i,1),
                 (*vertices)(i,2),
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


//implements picking of one of the coordinate axes
int pick_axes(const Vector3 &point, //3d point corresponding to the center of the coordinate frame
	const double length, //length of the arrows
	const double radius, //radius of the arrows
	const int &mouse_x, //mouse location
	const int &mouse_y, //mouse location
	Camera camera,
	const int w_x,// = 1,  // optional: width of window around the mouse in which we will look for the selected axes
	const int w_y)// = 1  // optional: height of window around the mouse in which we will search for the selected axes
{
	return pick_axes(
		point,
		length,
		radius,
		mouse_x,
		mouse_y,
		camera.m_viewport,
		camera.m_projection_matrix,
		camera.m_modelview_matrix,
		w_x,
		w_y);
}

//implements picking of a face and returns the index of the vertex in the face that was closest to the mouse point
int pick_face_and_vertex(const int &mouse_x, // mouse location
	const int &mouse_y,
	Camera camera,
	const Eigen::MatrixXd *vertices, //list of vertices
	const Eigen::MatrixXi *faces, //list of faces
	std::vector< std::pair<double, IndexType> > &hit_face_list, // output: list of hit faces
	const std::vector<int > &face_indices,// = std::vector<int>(0), // optional: list of face indices within faces that should be tested for picking. 
	const std::vector<int > &face_labels,// = std::vector<int>(0), // optional: labels of faces that should be tested for picking. Should be of the same size as face_indices.
	const int w_x,// = 1, // optional: width of window around the mouse in which we will search for faces
	const int w_y,// = 1, // optional: height of window around the mouse in which we will search for faces
	const int *center_x,// = 0, // optional: center of the window around the mouse in which we will search for faces. If not provided, the mouse position will be used.
	const int *center_y)// = 0)
{
	return pick_face_and_vertex(
		mouse_x,
		mouse_y,
		camera.m_viewport,
		camera.m_projection_matrix,
		camera.m_modelview_matrix,
		vertices,
		faces,
		hit_face_list,
		face_indices,
		face_labels,
		w_x,
		w_y,
		center_x,
		center_y
		);
}

int pick_nearest_vertex(const int &mouse_x, // mouse location
	const int &mouse_y,
	double &minsqdist,//the sq dist of nearest handle
	Camera camera,
	const Eigen::MatrixXd *vertices, //list of vertices
	const int w_x,// = 1, // optional: width of window around the mouse in which we will search for faces
	const int w_y,// = 1, // optional: height of window around the mouse in which we will search for faces
	const int *center_x,// = 0, // optional: center of the window around the mouse in which we will search for faces. If not provided, the mouse position will be used.
	const int *center_y)// = 0)
{
	return pick_nearest_vertex(
		mouse_x,
		mouse_y,
		minsqdist,
		camera.m_viewport,
		camera.m_projection_matrix,
		camera.m_modelview_matrix,
		vertices,
		w_x,
		w_y,
		center_x,
		center_y);
}

void computeRegionCentroid(const Eigen::MatrixXd *vertices,
	const std::vector<int> &labels,
	int r,
	RowVector3 &centroid)
{
	centroid = RowVector3::Zero();
	int num = 0;
	for (long vi = 0; vi<vertices->rows(); ++vi)
		if(labels[vi]==r)
		{
			centroid += vertices->row(vi);
			num++;
		}
		centroid/=num;
}

void computeRegionBoundingBox(const Eigen::MatrixXd *vertices,
	const std::vector<int> &labels,
	int r,
	RowVector3 &aabb,
	RowVector3 &AABB)
{
	aabb = RowVector3::Constant(vertices->maxCoeff());
	AABB = RowVector3::Constant(vertices->minCoeff());
	for (long vi = 0; vi<vertices->rows(); ++vi)
		if(labels[vi]==r)
		{
			aabb = aabb.cwiseMin(vertices->row(vi));
			AABB = aabb.cwiseMax(vertices->row(vi));
		}
}



//bfs for connected components
void bfs(const std::vector<int> &vertices, const int start_index, const std::vector< std::vector<IndexType > > &edges, int r, std::vector<int> &V )
{
	int start_vertex = vertices[start_index];
	std::queue<int> Q;
	Q.push(start_vertex);
	V[start_index] = r;
	while(!Q.empty())
	{
		int i = Q.front();
		// get the tail element from queue
		Q.pop();
		for(std::vector<unsigned int>::const_iterator it = edges[i].begin(); it != edges[i].end(); ++it)
		{
			unsigned long ii = std::find(vertices.begin(), vertices.end(), *it) - vertices.begin();
			if(ii<vertices.size() && !V[ii])
			{
				V[ii] = r;
				Q.push(vertices[ii]);
			}
		}
	}
}

//predicate for determining if a picked vertex has already been assigned a component
bool is0 (int i)
{
	return !(i);
}

//connected components on the selected vertices
int connected_components (const std::vector<int> &vertices, // indices of the selected vertices
	const std::vector<std::vector<IndexType > > &edges, // neighbors of *all* the vertices (i.e. also contains neighborhood info for vertices outside the selected set, and edges[i] != edges[vertices[i]])
	std::vector<int> &component //connected component assigned to each vertex (1-based, 0 indicates no assigned component)
	)
{
	long index = 0;
	unsigned long start_index = 0;
	while (start_index<vertices.size())
	{
		bfs(vertices,start_index, edges, ++index, component);
		std::vector<int >::iterator it = find_if (component.begin(), component.end(), is0);
		start_index = it - component.begin();
	}
	return index;

}



void adjacency_list(
	const Eigen::MatrixXi& F,
	std::vector<std::vector<IndexType> >& A)
{

	A.clear();
	if(F.rows()>0)//wangyu added this to enable zero size F
	{
		A.resize(F.maxCoeff()+1);
	}
	else
	{
		return;
	}

	// Loop over faces
	for(int i = 0;i<F.rows();i++)
	{
		// Loop over this face
		for(int j = 0;j<F.cols();j++)
		{
			// Get indices of edge: s --> d
			int s = F(i,j);
			int d = F(i,(j+1)%F.cols());
			A.at(s).push_back(d);
			A.at(d).push_back(s);
		}
	}

	// Remove duplicates
	for(int i=0; i<(int)A.size();++i)
	{
		std::sort(A[i].begin(), A[i].end());
		A[i].erase(std::unique(A[i].begin(), A[i].end()), A[i].end());
	}
}