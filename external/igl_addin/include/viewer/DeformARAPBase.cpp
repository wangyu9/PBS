#include "DeformARAPBase.h"


DeformARAPBase::DeformARAPBase()
	:arapSolver()
{

	run_solver = false;
	is_mesh_loaded = false;
}

DeformARAPBase::~DeformARAPBase()
{
}

void DeformARAPBase::InitMesh( const Eigen::MatrixXd& V, const Eigen::MatrixXi& T, const Eigen::MatrixXi& F)
{
	mesh_V = mesh_VR = V;
	mesh_T = T;
	mesh_F = F;
	is_mesh_loaded = true;
}

bool DeformARAPBase::SetConstraint(const LinearConstraint23d& lc)
{
	// TODO
	return true;
}

bool DeformARAPBase::SetConstraint(const Eigen::VectorXi& known, const Eigen::MatrixXd& knownPos, const Eigen::MatrixXd& Meq, const Eigen::MatrixXd& Peq)
{
	this->known = known;
	this->knownPos = knownPos;
	this->Meq = Meq;
	this->Peq = Peq;
	return arap_solver_set_constraint();
}

void DeformARAPBase::UpdateSolverOnce()
{
	assert(is_mesh_loaded);
	arap_solver_update();
}

void DeformARAPBase::Update()
{
	if(is_mesh_loaded && run_solver)
		arap_solver_update();
}

void DeformARAPBase::RestartSolver()
{
	arap_solver_init();
}

/************Start of multigrid related***********/

bool DeformARAPBase::arap_solver_init()
{
	arapSolver.init( mesh_VR, mesh_TF());
	return true;
}

bool DeformARAPBase::arap_solver_update()
{
	arapSolver.update();
	arapSolver.getV3d(mesh_V);
	return true;
}

bool DeformARAPBase::arap_solver_set_constraint()
{
	arapSolver.setConstraint(known, knownPos);
	return true;
}
/************ End  of multigrid related***********/