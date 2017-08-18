#ifndef MESH_DATA_H
#define MESH_DATA_H

#   ifdef _WIN32
#       include <windows.h>
#   endif

#include <vector>
#include "types.h"


class MeshData
{
public:
	// To make these private later.
	 
	/********* Mesh Data *********/
	
	// Vertices of the current mesh (#V x 3)
	// The i-th row contains the 3d coordinates of the i-th vertex
	Eigen::MatrixXd *vertices;
	// Face list of current mesh    (#F x 3) or (#F x 4)
	// The i-th row contains the indices of the vertices that forms the i-th face in ccw order (triangles only)
	Eigen::MatrixXi *faces;
	Eigen::MatrixXd rest_vertices;//added by wangyu
	Eigen::MatrixXi *tets;//added by wangyu

public:
	
	float mesh_alpha;//added by wangyu
	// Face Neighbors
	std::vector<std::vector<IndexType> > vertex_to_faces;
	// Vertex Neighbors
	std::vector<std::vector<IndexType> > vertex_to_vertices;
	// Corner normals
	Eigen::MatrixXd * corner_normals;
	// list of face indices into corner normals
	Eigen::MatrixXi *fNormIndices;
	// One normal per face
	Eigen::MatrixXd * face_normals;
	// One color per face
	Eigen::MatrixXd * face_colors;
	// One scalar per face
	VectorX *face_property;
	// One normal per vertex
	Eigen::MatrixXd * vertex_normals;
	// One color per vertex
	Eigen::MatrixXd * vertex_colors;
	// One scalar per vertex
	VectorX *vertex_property;
	// Texture coordinates
	Eigen::MatrixXd *texCoords;
	// List of face indices into TC
	Eigen::MatrixXi *fTexIndices;
	// Per-face flag to enable/disable texturing of the face
	std::vector<bool> fUseTexture;

	// number of vertices in mesh should == vertices->rows()
	// This field is used only to visualize the number of vertices in the
	// AntTweakBar
	size_t number_of_vertices;
	// number of faces in mesh should == faces->rows()
	// This field is used only to visualize the number of faces in the
	// AntTweakBar
	size_t number_of_faces;
	size_t number_of_tets;//wangyu


	/*********************************/

	/******** Overlays ***************/
	// Lines plotted over the scene 
	// (Every row contains 9 doubles in the following format S_x, S_y, S_z, T_x, T_y, T_z, C_r, C_g, C_b),
	// with S and T the coordinates of the two vertices of the line in global coordinates, and C the color in floating point rgb format
	std::vector<std::vector<double > > lines;
	// Points plotted over the scene
	// (Every row contains 7 doubles in the following format P_x, P_y, P_z, size, C_r, C_g, C_b),
	// with P the position in global coordinates of the center of the point, size its dimension in screen pixels, and C the color in floating point rgb format
	std::vector<std::vector<double > > points;
	// Text labels plotted over the scene
	// Textp contains, in the i-th row, the position in global coordinates where the i-th label should be anchored
	// Texts contains in the i-th position the text of the i-th label
	std::vector<std::vector<double > > textp;
	std::vector<std::string > texts;
	/*********************************/

	std::string filename;

	MeshData();
	~MeshData();

	bool load_texture_coordinates_from_file(const char* file_name);
	bool load_mesh_from_file(const char* mesh_file_name);
	bool load_pose_mesh_from_file(const char* mesh_file_name);
	bool save_mesh_to_file(const char* mesh_file_name);
	void save_tets_to_file(const char* mesh_file_name);//added by wangyu
	void clear_mesh();

	// TODO: make these const reference
	const Eigen::MatrixXd GetV() const;
	const Eigen::MatrixXi GetT() const;
	const Eigen::MatrixXi GetF() const;
	const Eigen::MatrixXd GetVR() const;

	void SetV(const Eigen::MatrixXd& VV);
	void SetT(const Eigen::MatrixXi& TT);
	void SetF(const Eigen::MatrixXi& FF);

};

#endif /*MESH_DATA_H*/