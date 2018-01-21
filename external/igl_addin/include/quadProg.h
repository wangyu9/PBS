#ifndef IGL_ADDIN_QUADPROG
#define IGL_ADDIN_QUADPROG

#include <Eigen/Core>
#include <Eigen/Sparse>

#include <igl/igl_inline.h>

//template <typename DerivedQ, typename Derivedf, typename DerivedAeq, typename DerivedBeq, typename DerivedZ>
//IGL_INLINE bool quadprog(
//	const Eigen::PlainObjectBase<DerivedQ>& Q,
//	const Eigen::PlainObjectBase<Derivedf> & f,
//	const Eigen::PlainObjectBase<DerivedAeq>& Aeq,
//	const Eigen::PlainObjectBase<DerivedBeq> & Beq,
//	Eigen::PlainObjectBase<DerivedZ> & Z);

#ifdef USE_MATLAB

#include <igl/matlab/matlabinterface.h>

inline bool quadprog(
	Engine **matlabEngine,
	const Eigen::MatrixXd & Q,
	const Eigen::MatrixXd & f,
	const Eigen::MatrixXd & Aeq,
	const Eigen::MatrixXd & Beq,
	Eigen::MatrixXd & Z)
{
	using namespace igl::matlab;
	//Engine **matlabEngine = new (Engine*);
	//igl::mlinit(matlabEngine);

	mleval(matlabEngine,"clear;");

	mlsetmatrix(matlabEngine, "Q", Q);
	mlsetmatrix(matlabEngine, "f", f);
	mlsetmatrix(matlabEngine, "Aeq", Aeq);
	mlsetmatrix(matlabEngine, "Beq", Beq);

	mleval(matlabEngine,"[Z] = min_quad_with_fixed(0.5*Q,f,[],[],Aeq,Beq);");
	// This should be updated for the new version of gptoolbox as 1.0*Q
	//igl::mleval(matlabEngine,"[Z, val, exitflag] = quadprog(Q,f,[],[],Aeq,Beq,[],[],[],[]);");

	mlgetmatrix(matlabEngine, "Z", Z);

	//igl::mlclose(matlabEngine);

	return true;
}

#endif

#endif