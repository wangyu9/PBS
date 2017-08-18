#include "arap_solver.h"

ARAPSolver::ARAPSolver()
	: dim(2), has_inited(false)
{
	solver= new GeometrySolver();
}

ARAPSolver::~ARAPSolver()
{
}

#define CROP_TO_2D_IF_NEEDED(V,d) (V.cols()== 3&&d==2)?V.leftCols<2>():V


void ARAPSolver::init(const Eigen::MatrixXd& V, const Eigen::MatrixXi& TF)
{

	if (TF.cols() != 3 && TF.cols() != 4)
	{
		std::cerr<<"ARAPSolver::Init() fails on TF!"<<std::endl;
		return;
	}
	assert(TF.cols() == 3 || TF.cols() == 4);

	dim = TF.cols() - 1;

	assert(V.cols() == 3 || V.cols() == 2 && dim == 2);

	const Eigen::MatrixXd VV = CROP_TO_2D_IF_NEEDED(V,dim);

	{
		ARAP_para ap;

		ap.var = CROP_TO_2D_IF_NEEDED(V,dim);

		ap.with_subspace_ARAP = false;
		ap.use_sparse_weights = false;

		if (ap.with_subspace_ARAP)
		{
			if (ap.use_sparse_weights)
			{
				assert(false);
				ap.subspace_M_sparse;// = infos[i]->bases_sparse;
			} 
			else
			{
				assert(false);
				ap.subspace_M;// = infos[i]->bases;
			}
			
			assert(false);
			ap.rotation_group;// = infos[i]->rotation_group;//ARAP_rotation_group;
		}

		ap.dim = dim;

		ap.with_dynamics = false;

		if (dim == 2)
			ap.mesh = ARAP_mesh(VV, Eigen::MatrixXi(0, 4), TF);
		else
			ap.mesh = ARAP_mesh(VV, TF, Eigen::MatrixXi(0, 3) );

		ap.phys_para = ARAP_phys_para(1.,1.,1.);//(time_step, mu, rho);

		// ap.per_group_weight = ARAP_per_group_weight; // it is OK for not settign this one

		solver->init(ap);

	}

	has_inited = true;
}

void ARAPSolver::update()
{
	double gravity[3] = { 0.,0.,0. };
	{
		solver->update(1, 1, true, gravity, 1.);
	}
}

void ARAPSolver::getV(Eigen::MatrixXd& VV)
{
	auto tmp = solver->getV();
	VV = tmp;
}

void ARAPSolver::getV3d(Eigen::MatrixXd& VV)
{
	auto tmp = solver->getV();
	VV.resize(tmp.rows(),3);
	VV.leftCols(tmp.cols()) = tmp;
}

#include <eigen_helper.h>
void ARAPSolver::setConstraint(const Eigen::VectorXi& known, const Eigen::MatrixXd& knownPos)
{
	assert(knownPos.cols() == 3 || knownPos.cols() == 2 && dim == 2);

	// Preprocessing
	const Eigen::MatrixXd kP = CROP_TO_2D_IF_NEEDED(knownPos,dim);//(knownPos.cols() == 3 && dim == 2) ? knownPos.leftCols<2>() : knownPos;

	if (!has_inited)
	{
		std::cout << "setConstraint(): ignored since not initialized!" << std::endl;
		return;
	}

	{
		solver->setConstraints(known, kP);
	}
}

