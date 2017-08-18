#pragma once

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

#include <Eigen/Core>
#include <vector>
#include "handle.h"

// From Preview3D
Eigen::Vector3f screen_to_world(float x, float y, float z, double m_modelview_matrix[16], double m_projection_matrix[16], GLint m_viewport[4]);

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
	bool *selected_axis =0  //if this is set, we will only rotate along the axis i for which selected_axis[i] = true 
	);

void get_translation(
	int mouse_x,
	int from_x,
	int mouse_y,
	int from_y,
	double depth,
	float *translation);

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
	bool *selected_axis =0 //if this is set, we will only rotate along the axis i for which selected_axis[i] = true 
	);



int pick_nearest_handle(const int &mouse_x, // mouse location
	const int &mouse_y,
	double &minsqdist,//the sq dist of nearest handle
	GLint *viewport, //viewport
	const double *projection_matrix, //opengl projection matrix
	const double *modelview_matrix, //opengl modelview matrix
	const std::vector<handleData> &handle_list, //list of vertices
	const int w_x = 1, // optional: width of window around the mouse in which we will search for faces
	const int w_y = 1, // optional: height of window around the mouse in which we will search for faces
	const int *center_x = 0, // optional: center of the window around the mouse in which we will search for faces. If not provided, the mouse position will be used.
	const int *center_y = 0);

int pick_nearest_point(const int &mouse_x, // mouse location
	const int &mouse_y,
	double &minsqdist,//the sq dist of nearest handle
	GLint *viewport, //viewport
	const double *projection_matrix, //opengl projection matrix
	const double *modelview_matrix, //opengl modelview matrix
	const Eigen::MatrixXd& V, // points matrix to search in, #V by 3
	const int w_x = 1, // optional: width of window around the mouse in which we will search for faces
	const int w_y = 1, // optional: height of window around the mouse in which we will search for faces
	const int *center_x = 0, // optional: center of the window around the mouse in which we will search for faces. If not provided, the mouse position will be used.
	const int *center_y = 0);

bool ray_trace_mesh(const int mouse_x, const int mouse_y, double& x, double& y, double& z, int& hit_index); // mouse location

bool ray_trace_mesh(
	int & minZ_vertex,
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
	const int condidate_size = 10,//const double cursor_search_radius,
	const std::vector<int > &face_indices = std::vector<int> (0), // optional: list of face indices within faces that should be tested for picking. 
	const std::vector<int > &face_labels = std::vector<int> (0), // optional: labels of faces that should be tested for picking. Should be of the same size as face_indices.
	const int w_x = 5, // optional: width of window around the mouse in which we will search for faces
	const int w_y = 5, // optional: height of window around the mouse in which we will search for faces
	const int *center_x = 0, // optional: center of the window around the mouse in which we will search for faces. If not provided, the mouse position will be used.
	const int *center_y = 0);





//void ConvertQuaternionToMatrix(const float *quat, float *mat);

void ConvertQuaternionToMatrix4x4(const float *quat, Eigen::MatrixXd& Mat);

void ConvertQuaternionToMatrix3x3(const float *quat, Eigen::MatrixXd& Mat);

void rotate_around_axis(const double v0[3], const double center[3], const double axis[3], const double angle, double v1[3]);
//
//void rotate_selected_handles_around_axis(const double axis[3], const bool around_center, const double cursor_rotation_center[3], const double rotation_2D, Eigen::MatrixXd rest_Handles, std::vector<handleData>& all_handle_list, std::vector<int>& select_handle_list);