#pragma once

#include <geometrySolver.h>
#include <vector>


class ARAPSolver {
private:
	int dim;
	GeometrySolver* solver;
	bool has_inited;
public:
	ARAPSolver();// (int N);
	~ARAPSolver();
	void init(const Eigen::MatrixXd& V, const Eigen::MatrixXi& TF);
	void update();
	void setConstraint(const Eigen::VectorXi& known, const Eigen::MatrixXd& knownPos);
	void getV(Eigen::MatrixXd& VV);// this could have 2D or 3D columns
	void getV3d(Eigen::MatrixXd& VV);
};
