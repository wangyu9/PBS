#ifndef LINEAR_CONSTRAINT_H
#define LINEAR_CONSTRAINT_H

class LinearConstraint
{
public:
	Eigen::VectorXi known;
	Eigen::MatrixXd knownValue;
	Eigen::MatrixXd Meq;
	Eigen::MatrixXd Peq;
};

class LinearConstraint23d
{
public:
	LinearConstraint LC2d;
	LinearConstraint LC3d;
};

#endif /*LINEAR_CONSTRAINT_H*/