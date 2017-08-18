// IGL Viewer - Copyright (c) 2013 ETH Zurich. All rights reserved.
// Helper functions to load a texture from a TGA file and convert it to a format compatible with OpenGL

#ifndef viewer_texture_h
#define viewer_texture_h

#ifdef _WIN32
#include <GL/glew.h>
#endif

#include <vector>
#include <Eigen/Core>
//#include "tga.h"
#include "YImage.hpp"

using namespace std;

typedef std::vector<double> Vector;

bool max_cols(const vector<Vector > &A,
              Vector & max_A);
bool min_cols(const vector<Vector > &A,
              Vector & min_A);

GLuint texture_from_png(const string tga_file);

YImage* read_png(const std::string tga_file);

//template<typename ScalarType>
//bool flipCoord( const Eigen::Matrix<ScalarType, Eigen::Dynamic,2,Eigen::RowMajor> *A,
//               const int k,
//               Eigen::Matrix<ScalarType, Eigen::Dynamic,2,Eigen::RowMajor> *B);
//template<typename ScalarType>
//bool copyXY(const Eigen::Matrix<ScalarType, Eigen::Dynamic,3,Eigen::RowMajor>  *A,
//            Eigen::Matrix<ScalarType, Eigen::Dynamic,2,Eigen::RowMajor>  *B);
//
//template<typename ScalarType>
//void normalize(const Eigen::Matrix<ScalarType, Eigen::Dynamic,2,Eigen::RowMajor> *A,
//               Eigen::Matrix<ScalarType, Eigen::Dynamic,2,Eigen::RowMajor> *B);


bool flipCoord(const Eigen::MatrixXd *A,
	const int k,
	Eigen::MatrixXd *B);

bool copyXY(const Eigen::MatrixXd  *A,
	Eigen::MatrixXd  *B);

void normalize(const Eigen::MatrixXd *A,
	Eigen::MatrixXd *B);

// compute max value in A
double maxv(const Vector &A);

// compute min value in A
double minv(const Vector &A);

bool normalized(const Eigen::MatrixXd *A);


#endif
