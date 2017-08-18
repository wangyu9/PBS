//
//  types.h
//  Preview3D
//
//  Created by Olga Diamanti on 3/2/12.
//  Copyright (c) 2012 ETH Zurich. All rights reserved.
//

#ifndef types_h
#define types_h

#include <complex>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>

#define GL_INDEX_TYPE GL_UNSIGNED_INT
// Must be one of GL_UNSIGNED_BYTE, GL_UNSIGNED_SHORT, or GL_UNSIGNED_INT

#define GL_SCALAR_TYPE GL_DOUBLE
// Must be one of GL_SHORT, GL_INT, GL_FLOAT, or GL_DOUBLE


#if GL_INDEX_TYPE == GL_UNSIGNED_INT
typedef unsigned int IndexType; 
#elif GL_INDEX_TYPE == GL_UNSIGNED_SHORT
typedef unsigned short IndexType; 
#elif GL_INDEX_TYPE == GL_UNSIGNED_BYTE
typedef unsigned char IndexType; 
#else
#ifdef GL_INDEX_TYPE
#undef GL_INDEX_TYPE
#endif
#define GL_INDEX_TYPE GL_UNSIGNED_INT
typedef unsigned int IndexType; 
#endif

#if GL_SCALAR_TYPE == GL_DOUBLE
typedef double ScalarType;
#elif GL_SCALAR_TYPE == GL_FLOAT
typedef float ScalarType;
#elif GL_SCALAR_TYPE == GL_SHORT
typedef short ScalarType;
#elif GL_SCALAR_TYPE == GL_INT
typedef int ScalarType;
#else
#ifdef GL_SCALAR_TYPE
#undef GL_SCALAR_TYPE
#endif
#define GL_SCALAR_TYPE GL_FLOAT
typedef float ScalarType;
#endif

#define  EIGEN_DONT_ALIGN_STATICALLY

//row major order required for VBO's
typedef Eigen::Matrix<ScalarType, Eigen::Dynamic, 3,              Eigen::RowMajor> PointMatrixType;
typedef Eigen::Matrix<IndexType,  Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> FaceMatrixType;
typedef Eigen::Matrix<ScalarType, Eigen::Dynamic, 2,              Eigen::RowMajor> UVMatrixType;

typedef Eigen::Matrix<ScalarType, Eigen::Dynamic, Eigen::Dynamic> MatrixXX;
typedef Eigen::Matrix<ScalarType, Eigen::Dynamic, 2>              MatrixX2;
typedef Eigen::Matrix<ScalarType, Eigen::Dynamic, 3>              MatrixX3;
typedef Eigen::Matrix<ScalarType, 3,              Eigen::Dynamic> Matrix3X;
typedef Eigen::Matrix<ScalarType, Eigen::Dynamic, 8>              MatrixX8;
typedef Eigen::Matrix<ScalarType, 3,              3>              Matrix33;
typedef Eigen::Matrix<ScalarType, 4,              4>              Matrix44;
typedef Eigen::Matrix<ScalarType, 2,              8>              Matrix28;

typedef Eigen::Matrix<ScalarType, Eigen::Dynamic, 1> VectorX;
typedef Eigen::Matrix<ScalarType, 3,              1> Vector3;
typedef Eigen::Matrix<ScalarType, 2,              1> Vector2;

typedef Eigen::Matrix<ScalarType, 1,              Eigen::Dynamic> RowVectorX;
typedef Eigen::Matrix<ScalarType, 1,              8>              RowVector8;
typedef Eigen::Matrix<ScalarType, 1,              3>              RowVector3;
typedef Eigen::Matrix<ScalarType, 1,              2>              RowVector2;
typedef Eigen::Matrix<ScalarType, 1,              4>              RowVector4;

typedef Eigen::Matrix<IndexType,  Eigen::Dynamic, Eigen::Dynamic> MatrixXXi;
typedef Eigen::Matrix<IndexType,  Eigen::Dynamic, 3>              MatrixX3i;
typedef Eigen::Matrix<IndexType,  Eigen::Dynamic, 4>              MatrixX4i;

typedef Eigen::Matrix<IndexType,  Eigen::Dynamic, 1> VectorXi;
typedef Eigen::Matrix<IndexType,  3,              1> Vector3i;

typedef Eigen::Matrix<IndexType,  1,              3> RowVector3i;

typedef Eigen::Quaternion<double, Eigen::DontAlign> Quatd;

#ifndef DEGREE2RADIAN
#define DEGREE2RADIAN(degree) ((degree)*M_PI/180.0)
#endif
#ifndef RADIAN2DEGREE
#define RADIAN2DEGREE(radian) ((radian)*180.0/M_PI)
#endif

#ifndef SAFE_DELETE
#define SAFE_DELETE(ptr)
#endif
#undef SAFE_DELETE
#define SAFE_DELETE(ptr) \
	if (ptr) { delete ptr; ptr = NULL; }

#ifndef MYRELEASEASSERT
#define MYRELEASEASSERT(x) \
{ \
	if(!(x)) \
{ \
	std::cout<<"Assert Failed: "<<__FILE__<<", "<<__LINE__<<" line, "<<#x<<"\n"; \
{__asm int 3} \
} \
}
#endif

#define SQRT2_2 0.7071067811865475244
#ifndef M_TWO_PI
#define M_TWO_PI 6.2831853071795864769
#endif
#ifndef M_PI_2
#define M_PI_2     1.57079632679489661923
#endif
#define  GOLDEN_CONST 0.61803398874
#define  GOLDEN_CONST_2 0.38196601126

//#define  DOTIMING
#ifdef DOTIMING
#include "../../../../../../lib/3rd/include/roy/profile/profile.h"
#endif

#endif

