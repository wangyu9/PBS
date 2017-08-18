#ifndef IGL_ADDIN_PICKING_UTIL
#define IGL_ADDIN_PICKING_UTIL


#ifdef __APPLE__
#   include <OpenGL/gl.h>
#   include <OpenGL/glu.h>
#   include <GLUT/glut.h>
#else
#   ifdef _WIN32
#       define NOMINMAX // disables min/max marco of WinDef.h
#       include <windows.h>
#       include <GL/glew.h>
#       include <GL/glut.h>
#   endif
#   include <GL/gl.h>
#   include <GL/glu.h>
#endif

#include <igl/igl_inline.h>
#include <vector>
#include "types.h"

#include <Camera.h>

//implements picking of one of the coordinate axes
int pick_axes(const Vector3 &point, //3d point corresponding to the center of the coordinate frame
                     const double length, //length of the arrows
                     const double radius, //radius of the arrows
                     const int &mouse_x, //mouse location
                     const int &mouse_y, //mouse location
                     GLint *viewport, //opengl viewport
                     const double *projection_matrix, //opengl projection matrix
                     const double *modelview_matrix, //opengl modelview matrix
                     const int w_x = 1,  // optional: width of window around the mouse in which we will look for the selected axes
                     const int w_y = 1  // optional: height of window around the mouse in which we will search for the selected axes
                     );

//implements picking of a face and returns the index of the vertex in the face that was closest to the mouse point
int pick_face_and_vertex(const int &mouse_x, // mouse location
                                const int &mouse_y,
                                GLint *viewport, //viewport
                                const double *projection_matrix, //opengl projection matrix
                                const double *modelview_matrix, //opengl modelview matrix
                                const PointMatrixType *vertices, //list of vertices
                                const FaceMatrixType *faces, //list of faces
                                std::vector< std::pair<double,IndexType> > &hit_face_list, // output: list of hit faces
                                const std::vector<int > &face_indices = std::vector<int> (0), // optional: list of face indices within faces that should be tested for picking. 
                                const std::vector<int > &face_labels = std::vector<int> (0), // optional: labels of faces that should be tested for picking. Should be of the same size as face_indices.
                                const int w_x = 1, // optional: width of window around the mouse in which we will search for faces
                                const int w_y = 1, // optional: height of window around the mouse in which we will search for faces
                                const int *center_x = 0, // optional: center of the window around the mouse in which we will search for faces. If not provided, the mouse position will be used.
                                const int *center_y = 0);

int pick_nearest_vertex(const int &mouse_x, // mouse location
	 const int &mouse_y,
	 double &minsqdist,//the sq dist of nearest handle
	 GLint *viewport, //viewport
	 const double *projection_matrix, //opengl projection matrix
	 const double *modelview_matrix, //opengl modelview matrix
	 const PointMatrixType *vertices, //list of vertices
	 const int w_x = 1, // optional: width of window around the mouse in which we will search for faces
	 const int w_y = 1, // optional: height of window around the mouse in which we will search for faces
	 const int *center_x = 0, // optional: center of the window around the mouse in which we will search for faces. If not provided, the mouse position will be used.
	 const int *center_y = 0);

//implements picking of one of the coordinate axes
int pick_axes(const Vector3 &point, //3d point corresponding to the center of the coordinate frame
	const double length, //length of the arrows
	const double radius, //radius of the arrows
	const int &mouse_x, //mouse location
	const int &mouse_y, //mouse location
	Camera camera,
	const int w_x = 1,  // optional: width of window around the mouse in which we will look for the selected axes
	const int w_y = 1  // optional: height of window around the mouse in which we will search for the selected axes
	);

//implements picking of a face and returns the index of the vertex in the face that was closest to the mouse point
int pick_face_and_vertex(const int &mouse_x, // mouse location
	const int &mouse_y,
	Camera camera,
	const PointMatrixType *vertices, //list of vertices
	const FaceMatrixType *faces, //list of faces
	std::vector< std::pair<double, IndexType> > &hit_face_list, // output: list of hit faces
	const std::vector<int > &face_indices = std::vector<int>(0), // optional: list of face indices within faces that should be tested for picking. 
	const std::vector<int > &face_labels = std::vector<int>(0), // optional: labels of faces that should be tested for picking. Should be of the same size as face_indices.
	const int w_x = 1, // optional: width of window around the mouse in which we will search for faces
	const int w_y = 1, // optional: height of window around the mouse in which we will search for faces
	const int *center_x = 0, // optional: center of the window around the mouse in which we will search for faces. If not provided, the mouse position will be used.
	const int *center_y = 0);

int pick_nearest_vertex(const int &mouse_x, // mouse location
	const int &mouse_y,
	double &minsqdist,//the sq dist of nearest handle
	Camera camera,
	const PointMatrixType *vertices, //list of vertices
	const int w_x = 1, // optional: width of window around the mouse in which we will search for faces
	const int w_y = 1, // optional: height of window around the mouse in which we will search for faces
	const int *center_x = 0, // optional: center of the window around the mouse in which we will search for faces. If not provided, the mouse position will be used.
	const int *center_y = 0);

void computeRegionCentroid(const PointMatrixType *vertices,
                                  const std::vector<int> &labels,
                                  int r,
                                  RowVector3 &centroid);

void computeRegionBoundingBox(const PointMatrixType *vertices,
                                     const std::vector<int> &labels,
                                     int r,
                                     RowVector3 &aabb,
                                     RowVector3 &AABB);


//bfs for connected components
void bfs(const std::vector<int> &vertices, const int start_index, const std::vector< std::vector<IndexType > > &edges, int r, std::vector<int> &V );

//predicate for determining if a picked vertex has already been assigned a component
bool is0 (int i) ;

//connected components on the selected vertices
int connected_components (const std::vector<int> &vertices, // indices of the selected vertices
	const std::vector<std::vector<IndexType > > &edges, // neighbors of *all* the vertices (i.e. also contains neighborhood info for vertices outside the selected set, and edges[i] != edges[vertices[i]])
	std::vector<int> &component //connected component assigned to each vertex (1-based, 0 indicates no assigned component)
	);


////determine whether (x,y) is inside/outside of a polyline
////1=inside, 0=outside
//// from http://www.visibone.com/inpoly/
//int inpoly ( const std::vector<std::vector<unsigned int > >&poly, //polygon points, [0]=x, [1]=y. Polyline need not be closed (i.e. first point != last point), the line segment between last and first selected points is constructed within this function.
//	const unsigned int xt, //x (horizontal) of target point
//	const unsigned int yt) //y (vertical) of target point
//	;

//determine whether (x,y) is inside/outside of a polyline
//1=inside, 0=outside
// from http://www.visibone.com/inpoly/
static int inpoly ( const std::vector<std::vector<unsigned int > >&poly, //polygon points, [0]=x, [1]=y. Polyline need not be closed (i.e. first point != last point), the line segment between last and first selected points is constructed within this function.
	const unsigned int xt, //x (horizontal) of target point
	const unsigned int yt) //y (vertical) of target point
{
	int npoints= poly.size();
	unsigned int xnew,ynew;
	unsigned int xold,yold;
	unsigned int x1,y1;
	unsigned int x2,y2;
	int i;
	int inside=0;

	if (npoints < 3) {
		return(0);
	}
	xold=poly[npoints-1][0];
	yold=poly[npoints-1][1];
	for (i=0 ; i < npoints ; i++) {
		xnew=poly[i][0];
		ynew=poly[i][1];
		if (xnew > xold) {
			x1=xold;
			x2=xnew;
			y1=yold;
			y2=ynew;
		}
		else {
			x1=xnew;
			x2=xold;
			y1=ynew;
			y2=yold;
		}
		if ((xnew < xt) == (xt <= xold)          /* edge "open" at one end */
			&& ((long)yt-(long)y1)*(long)(x2-x1)
			< ((long)y2-(long)y1)*(long)(xt-x1)) {
				inside=!inside;
		}
		xold=xnew;
		yold=ynew;
	}
	return(inside);
}


void adjacency_list(
	const FaceMatrixType& F, 
	std::vector<std::vector<IndexType> >& A);



#ifdef IGL_HEADER_ONLY
#	include "picking_util.cpp"
#endif

#endif /*IGL_ADDIN_PICKING_UTIL*/