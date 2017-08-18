#ifndef IGL_ADDIN_LBS
#define IGL_ADDIN_LBS

#include <igl/igl_inline.h>
#include <Eigen/Core>
#include "types.h"

void IGL_INLINE convertQuaternionToMatrix(double quaternion[4], Eigen::Matrix3Xd& rot_mat);

void IGL_INLINE lbs_trans_matrix(const PointMatrixType & H_rest, const PointMatrixType & H, const Eigen::MatrixXd & R, PointMatrixType & T);

void IGL_INLINE constraint_matrix_from_M(const Eigen::MatrixXd & M, const Eigen::VectorXi &clist, const Eigen::MatrixXd V, Eigen::MatrixXd & Meq, Eigen::MatrixXd & Peq);

#ifdef IGL_HEADER_ONLY
	#include "lbs.cpp"
#endif

#endif /*IGL_ADDIN_LBS*/