#ifndef PLANE_SYMMETRIC_H
#define PLANE_SYMMETRIC_H

Eigen::MatrixXd plane_symmetric(const Eigen::MatrixXd& P)
{
	Eigen::MatrixXd SP = P;
	if (SP.cols()>=1)
	{
		SP.col(0) = -SP.col(0);
	}

	return SP;
}

#endif /*PLANE_SYMMETRIC_H*/

