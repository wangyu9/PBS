#include "quadProg.h"

#ifdef USING_IGL_HEADER_ONLY_MODE
#define IGL_HEADER_ONLY 
#endif

#ifdef USE_MATLAB
#include <igl/matlab/matlabinterface.h>
#endif


//template <typename DerivedQ, typename Derivedf, typename DerivedAeq, typename DerivedBeq, typename DerivedZ>
//IGL_INLINE bool quadprog(
//	const Eigen::PlainObjectBase<DerivedQ>& Q,
//	const Eigen::PlainObjectBase<Derivedf> & f,
//	const Eigen::PlainObjectBase<DerivedAeq>& Aeq,
//	const Eigen::PlainObjectBase<DerivedBeq> & Beq,
//	Eigen::PlainObjectBase<DerivedZ> & Z)

//inline bool quadprog(
//	const Eigen::MatrixXd & Q,
//	const Eigen::MatrixXd & f,
//	const Eigen::MatrixXd & Aeq,
//	const Eigen::MatrixXd & Beq,
//	Eigen::MatrixXd & Z)
