#include "print_matlab.h"
#include <igl/matlab/matlabinterface.h>

IGL_INLINE void print_matlab(
	const Eigen::MatrixXd & M,
	const std::string& name)
{
	using namespace igl::matlab;

	Engine **matlabEngine = new (Engine*);
	mlinit(matlabEngine);

	//igl::mleval(matlabEngine,"clear;");

	mlsetmatrix(matlabEngine, name, M);

	//igl::mlclose(matlabEngine);
}

IGL_INLINE void print_matlab(
	const std::string& name,
	const Eigen::MatrixXd & M)
{
	print_matlab(M,name);
}

IGL_INLINE void print_matlab(
	const Eigen::MatrixXi & M,
	const std::string& name)
{
	using namespace igl::matlab;

	Engine **matlabEngine = new (Engine*);
	mlinit(matlabEngine);

	//igl::mleval(matlabEngine,"clear;");

	mlsetmatrix(matlabEngine, name, M);

	//igl::mlclose(matlabEngine);
}

IGL_INLINE void print_matlab(
	const std::string& name,
	const Eigen::MatrixXi & M)
{
	print_matlab(M,name);
}






//IGL_INLINE void print_matlab(
//	const Eigen::MatrixXd & M,
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
//
//}