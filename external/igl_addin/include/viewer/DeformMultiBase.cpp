#include "DeformMultiBase.h"


DeformMultiBase::DeformMultiBase()
	:multiGeometrySolver()
{

	run_solver = false;
	is_mesh_loaded = false;
}

DeformMultiBase::~DeformMultiBase()
{
}

void DeformMultiBase::InitMesh( const Eigen::MatrixXd& V, const Eigen::MatrixXi& T, const Eigen::MatrixXi& F)
{
	mesh_V = mesh_VR = V;
	mesh_T = T;
	mesh_F = F;
	is_mesh_loaded = true;
}

bool DeformMultiBase::SetConstraint(const LinearConstraint23d& lc)
{
	// TODO
	return true;
}

bool DeformMultiBase::SetConstraint(const Eigen::VectorXi& known, const Eigen::MatrixXd& knownPos, const Eigen::MatrixXd& Meq, const Eigen::MatrixXd& Peq)
{
	this->known = known;
	this->knownPos = knownPos;
	this->Meq = Meq;
	this->Peq = Peq;
	return multi_solver_set_constraint();
}

void DeformMultiBase::UpdateSolverOnce(int update_id, int get_id)
{
	assert(is_mesh_loaded);
	multi_solver_update(update_id,get_id);
}

void DeformMultiBase::Update(int update_id, int get_id)
{
	if(is_mesh_loaded && run_solver)
		multi_solver_update(update_id, get_id);
}

void DeformMultiBase::RestartSolver()
{
	multi_solver_init();
}

/************Start of multigrid related***********/

bool DeformMultiBase::multi_solver_init()
{
	std::vector<SubspaceInfo*> infos;

	generate_subspace_info(mesh_VR, mesh_TF(), known, infos);

	multiGeometrySolver.init( mesh_VR, mesh_TF(), infos);

	return true;
}

bool DeformMultiBase::multi_solver_update(int update_id, int get_id)
{
	//using namespace igl::matlab;
	//mleval(matlabEngine, "ExternalCall_update_deformer");

	//mlgetmatrix(matlabEngine, "VV", mesh_V);

	multiGeometrySolver.update(update_id);
	multiGeometrySolver.getV3d(get_id,mesh_V);

	return true;
}

bool DeformMultiBase::multi_solver_set_constraint()
{
	//using namespace igl::matlab;
	//using namespace Eigen;
	//
	//mlsetmatrix(matlabEngine, "known", MatrixXi(known) );
	//mlsetmatrix(matlabEngine, "knownPos", knownPos);
	//mlsetmatrix(matlabEngine, "Meq", Meq);
	//mlsetmatrix(matlabEngine, "Peq", Peq);

	//mleval(matlabEngine, "ExternalCall_set_constraint");

	multiGeometrySolver.setConstraint(known, knownPos);

	return true;
}

/************ End  of multigrid related***********/