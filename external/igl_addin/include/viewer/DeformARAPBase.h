#ifndef DEFORM_ARAP_BASE_H
#define DEFORM_ARAP_BASE_H

#include <Eigen/Dense>
#include <linear_constraint.h>

//#include <multi_geometry_solver.h>
#include <arap_solver.h>

class DeformARAPBase {
public:
	DeformARAPBase();
	~DeformARAPBase();

	void InitMesh(const Eigen::MatrixXd& V, const Eigen::MatrixXi& T, const Eigen::MatrixXi& F);

	bool SetConstraint(const LinearConstraint23d& lc);
	bool SetConstraint(const Eigen::VectorXi& known, const Eigen::MatrixXd& knownPos, const Eigen::MatrixXd& Meq, const Eigen::MatrixXd& Peq);

	bool run_solver;

	void UpdateSolverOnce();

	void Update();
	void RestartSolver();// use this to start and restart solver

	const Eigen::MatrixXd& GetMeshV() const { return mesh_V; }

private:
	bool is_mesh_loaded;

	Eigen::MatrixXd mesh_VR; // rest pose of the mesh

	Eigen::MatrixXd mesh_V; // deformed pose of the mesh
	Eigen::MatrixXi mesh_T;
	Eigen::MatrixXi mesh_F;

	inline Eigen::MatrixXi& mesh_TF() { return mesh_T.size() ? mesh_T : mesh_F; };


	Eigen::VectorXi known;
	Eigen::MatrixXd knownPos;
	Eigen::MatrixXd Meq;
	Eigen::MatrixXd Peq;

	/************Start of multigrid related***********/
	//MultiGeometrySolver multiGeometrySolver;
	ARAPSolver arapSolver;
	bool arap_solver_init();
	bool arap_solver_update();
	bool arap_solver_set_constraint();
	/*************End of multigrid related************/
};

#endif /*DEFORM_ARAP_BASE_H*/