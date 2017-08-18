#include "DeformMatlabBase.h"

DeformMatlabBase::DeformMatlabBase()
{
	matlab_init();

	run_solver = false;
	is_mesh_loaded = false;
}

DeformMatlabBase::~DeformMatlabBase()
{
	using namespace igl::matlab;
	if (*matlabEngine != NULL)
	{
		mlclose(matlabEngine);
		delete *matlabEngine;
	}
}

void DeformMatlabBase::InitMesh( const Eigen::MatrixXd& V, const Eigen::MatrixXi& T, const Eigen::MatrixXi& F)
{
	mesh_V = mesh_VR = V;
	mesh_T = T;
	mesh_F = F;
	is_mesh_loaded = true;
}

bool DeformMatlabBase::SetConstraint(const LinearConstraint23d& lc)
{
	// TODO
	return true;
}

bool DeformMatlabBase::SetConstraint(const Eigen::VectorXi& known, const Eigen::MatrixXd& knownPos, const Eigen::MatrixXd& Meq, const Eigen::MatrixXd& Peq)
{
	this->known = known;
	this->knownPos = knownPos;
	this->Meq = Meq;
	this->Peq = Peq;
	return matlab_solver_set_constraint();
}

void DeformMatlabBase::Update()
{
	if(is_mesh_loaded && run_solver)
		matlab_solver_update();
}

void DeformMatlabBase::RestartSolver()
{
	matlab_solver_init();
}

/************Start of Matlab related***********/
void DeformMatlabBase::matlab_init(const std::string MATLAB_FOLDER_PATH)
{
	using namespace igl::matlab;

	matlabEngine = new (Engine*);
	mlinit(matlabEngine);

	std::string str_cd_folder = std::string("cd ") + std::string(MATLAB_FOLDER_PATH);
	mleval(matlabEngine, str_cd_folder);
	// Clear the WorkSpace
	mleval(matlabEngine, "clear;");
}

bool DeformMatlabBase::matlab_solver_init()
{
	using namespace igl::matlab;

	/** Be careful that mlsetmatrix will add 1 for all interger type!**/

	mlsetmatrix(matlabEngine, "VR", mesh_VR);
	mlsetmatrix(matlabEngine, "T", mesh_T);
	mlsetmatrix(matlabEngine, "F", mesh_F);

	mleval(matlabEngine, "prepare_dependency;");
	mleval(matlabEngine, "ExternalCall_init_deformer;");

	return true;
}

bool DeformMatlabBase::matlab_solver_update()
{
	using namespace igl::matlab;
	mleval(matlabEngine, "ExternalCall_update_deformer");

	mlgetmatrix(matlabEngine, "VV", mesh_V);

	return true;
}

bool DeformMatlabBase::matlab_solver_set_constraint()
{
	using namespace igl::matlab;
	using namespace Eigen;
	
	mlsetmatrix(matlabEngine, "known", MatrixXi(known) );
	mlsetmatrix(matlabEngine, "knownPos", knownPos);
	mlsetmatrix(matlabEngine, "Meq", Meq);
	mlsetmatrix(matlabEngine, "Peq", Peq);

	mleval(matlabEngine, "ExternalCall_set_constraint");

	return true;
}
/************ End  of Matlab related***********/