#ifndef DEFORM_MATLAB_BASE_H
#define DEFORM_MATLAB_BASE_H

#include <Eigen/Dense>
#include <linear_constraint.h>

#include <igl/matlab/matlabinterface.h>

class DeformMatlabBase {
public:
	DeformMatlabBase();
	~DeformMatlabBase();

	void InitMesh(const Eigen::MatrixXd& V, const Eigen::MatrixXi& T, const Eigen::MatrixXi& F);

	bool SetConstraint(const LinearConstraint23d& lc);
	bool SetConstraint(const Eigen::VectorXi& known, const Eigen::MatrixXd& knownPos, const Eigen::MatrixXd& Meq, const Eigen::MatrixXd& Peq);

	bool run_solver;

	void Update();
	void RestartSolver();// use this to start and restart solver

	const Eigen::MatrixXd& GetMeshV() const { return mesh_V; }

private:
	bool is_mesh_loaded;

	Eigen::MatrixXd mesh_VR; // rest pose of the mesh

	Eigen::MatrixXd mesh_V; // deformed pose of the mesh
	Eigen::MatrixXi mesh_T;
	Eigen::MatrixXi mesh_F;

	Eigen::VectorXi known;
	Eigen::MatrixXd knownPos;
	Eigen::MatrixXd Meq;
	Eigen::MatrixXd Peq;

	/************Start of Matlab related***********/
	Engine **matlabEngine;
	void matlab_init(const std::string MATLAB_FOLDER_PATH);
	bool matlab_solver_init();
	bool matlab_solver_update();
	bool matlab_solver_set_constraint();
	/************ End  of Matlab related***********/
};

#endif /*DEFORM_MATLAB_BASE_H*/