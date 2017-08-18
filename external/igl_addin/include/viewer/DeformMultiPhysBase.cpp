#include "DeformMultiPhysBase.h"


DeformMultiPhysBase::DeformMultiPhysBase()
{

	run_solver = false;
	is_mesh_loaded = false;
}

DeformMultiPhysBase::~DeformMultiPhysBase()
{
}

void DeformMultiPhysBase::InitMesh( const Eigen::MatrixXd& V, const Eigen::MatrixXi& T, const Eigen::MatrixXi& F)
{
	mesh_V = mesh_VR = V;
	mesh_T = T;
	mesh_F = F;
	is_mesh_loaded = true;
}

bool DeformMultiPhysBase::SetConstraint(const LinearConstraint23d& lc)
{
	// TODO
	return true;
}

bool DeformMultiPhysBase::SetConstraint(const Eigen::VectorXi& known, const Eigen::MatrixXd& knownPos, const Eigen::MatrixXd& Meq, const Eigen::MatrixXd& Peq)
{
	this->known = known;
	this->knownPos = knownPos;
	this->Meq = Meq;
	this->Peq = Peq;
	return multi_solver_set_constraint();
}

void DeformMultiPhysBase::UpdateSolverOnce()
{
	assert(is_mesh_loaded);
	multi_solver_update();
}

void DeformMultiPhysBase::Update()
{
	if(is_mesh_loaded && run_solver)
		multi_solver_update();
}

void DeformMultiPhysBase::RestartSolver()
{
	multi_solver_init();
}

/************Start of multigrid related***********/

bool DeformMultiPhysBase::multi_solver_init()
{
	geometricElasticSolver.init( mesh_VR, mesh_TF());

	return true;
}

bool DeformMultiPhysBase::multi_solver_update()
{

	geometricElasticSolver.update();
	geometricElasticSolver.getV3d(mesh_V);

	return true;
}

bool DeformMultiPhysBase::multi_solver_set_constraint()
{

	geometricElasticSolver.setConstraint(known, knownPos);

	return true;
}

/************ End  of multigrid related***********/