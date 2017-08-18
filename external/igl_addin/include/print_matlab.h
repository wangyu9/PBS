#ifndef IGL_ADDIN_PRINT_MATLAB
#define IGL_ADDIN_PRINT_MATLAB

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <string>

#include <igl/igl_inline.h>

#include <igl/matlab/matlabinterface.h>


IGL_INLINE void print_matlab(
	const Eigen::MatrixXd & M,
	const std::string& name);

IGL_INLINE void print_matlab(
	const std::string& name,
	const Eigen::MatrixXd & M);

IGL_INLINE void print_matlab(
	const Eigen::MatrixXi & M,
	const std::string& name);

IGL_INLINE void print_matlab(
	const std::string& name,
	const Eigen::MatrixXi & M);
//
//IGL_INLINE void print_matlab(
//	const Eigen::MatrixXi & M,
//	const std::string& name)
//{
//	Engine **matlabEngine = new (Engine*);
//	igl::mlinit(matlabEngine);
//
//	//igl::mleval(matlabEngine,"clear;");
//
//	igl::mlsetmatrix(matlabEngine, name, M);
//
//	igl::mlclose(matlabEngine);
//}

#ifdef IGL_HEADER_ONLY
#include "print_matlab.cpp"
#endif

#endif /*IGL_ADDIN_PRINT_MATLAB*/