#include "lbs.h"

void IGL_INLINE convertQuaternionToMatrix(double quaternion[4], Eigen::Matrix3Xd& rot_mat)
{
	rot_mat = Eigen::MatrixXd::Zero(3,3);
	double &x = quaternion[0];
	double &y = quaternion[1];
	double &z = quaternion[2];
	double &w = quaternion[3];
	rot_mat(0,0) = 1 - 2*y*y - 2*z*z;
	rot_mat(0,1) = 2*x*y + 2*w*z;
	rot_mat(0,2) = 2*x*z - 2*w*y;
	rot_mat(1,0) = 2*x*y - 2*w*z;
	rot_mat(1,1) = 1 - 2*x*x - 2*z*z;
	rot_mat(1,2) = 2*y*z + 2*w*x;
	rot_mat(2,0) = 2*x*z + 2*w*y;
	rot_mat(2,1) = 2*y*z - 2*w*x;
	rot_mat(2,2) = 1 - 2*x*x - 2*y*y;
	return;
}

void IGL_INLINE lbs_trans_matrix(const Eigen::MatrixXd & H_rest, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R, Eigen::MatrixXd & T)
{
	assert( H.rows()==H_rest.rows() && H.cols()==H_rest.cols() );
	assert( H.cols()==R.cols() );
	// R is dim*m by dim
	int dim = R.cols();
	int m = H.rows();
	T.resize( (dim+1)*m, dim);

	for (int j=0; j<m; j++)
	{
		Eigen::MatrixXd Rj = R.block(dim*j, 0, dim, dim);
		T.block( (dim+1)*j, 0, dim, dim) = Rj.transpose();
		T.block( (dim+1)*j+dim, 0, 1, dim ) = H.block( j, 0, 1, dim) - H_rest.block( j, 0, 1, dim)*Rj.transpose();
	}
}

void IGL_INLINE constraint_matrix_from_M(const Eigen::MatrixXd & M, const Eigen::VectorXi &cList, const Eigen::MatrixXd cV, Eigen::MatrixXd & Meq, Eigen::MatrixXd & Peq)
{
	
}