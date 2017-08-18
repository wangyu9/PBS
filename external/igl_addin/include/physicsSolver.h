#ifndef PHYSICS_SOLVER_H
#define PHYSICS_SOLVER_H

#include <Eigen/Core>
#include <Eigen/Sparse>


class PhysicsSolver{
public:
	Eigen::MatrixXi T;
	Eigen::MatrixXd V0;//Initial vertices positions
	Eigen::MatrixXd V;//Current vertices positions
	Eigen::MatrixXd* D_m;
	Eigen::MatrixXd* B_m;
	Eigen::MatrixXd Forces;//Internal Elastic Forces
	double* W;//W[e] is the undeformed volume of T[e]

	Eigen::MatrixXd Vel;//Velocity

	//Eigen::MatrixXd C;//Constrained Vertex Coordinates
	Eigen::VectorXi S;//Indication of Constrained Vertex, 1:constrained, 0 not constrained
public:
	PhysicsSolver();
	void Init(Eigen::MatrixXi& TT,Eigen::MatrixXd& VV);
	void precomputation();
	void computeElasticForces(double mu);
	void computeForceDifferentials(double mu);
	void doTimeStep(double timeStep);
	void integrateExplicitEuler(double timeStep);
	void setConstraints(Eigen::VectorXi& SS, Eigen::MatrixXd& V_new);//only V_new with >= 0 value will be used
};


#endif PHYSICS_SOLVER_H