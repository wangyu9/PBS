#ifndef DEFORM_MULTI_BASE_H
#define DEFORM_MULTI_BASE_H

#include <Eigen/Dense>
#include <linear_constraint.h>

#include <multi_geometry_solver.h>


class DeformMultiBase {
public:
	DeformMultiBase();
	~DeformMultiBase();
	
	void InitMesh(const Eigen::MatrixXd& V, const Eigen::MatrixXi& T, const Eigen::MatrixXi& F);

	bool SetConstraint(const LinearConstraint23d& lc);
	bool SetConstraint(const Eigen::VectorXi& known, const Eigen::MatrixXd& knownPos, const Eigen::MatrixXd& Meq, const Eigen::MatrixXd& Peq);

	bool run_solver;

	void UpdateSolverOnce(int update_id = 0, int get_id = 0);

	void Update(int update_id = 0, int get_id = 0);
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
	MultiGeometrySolver multiGeometrySolver;

	bool multi_solver_init();
	bool multi_solver_update(int update_id, int get_id);
	bool multi_solver_set_constraint();

public:

	int GetMultiSolverNumLayer() const { return multiGeometrySolver.getNumLayer(); }
	/*************End of multigrid related************/
};

#endif /*DEFORM_MULTI_BASE_H*/