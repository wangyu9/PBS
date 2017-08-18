#include "multi_geometry_solver.h"

const int NUM_OF_MULTI_GRID_LEVEL = 5;

MultiGeometrySolver::MultiGeometrySolver()
	: numLayers(NUM_OF_MULTI_GRID_LEVEL), dim(2), has_inited(false)
{
	assert(numLayers>=2);
	// this is proved to be wrong
	//solvers.resize(numLayers);
	//for (auto s:solvers)
	//	s = new GeometrySolver();
	for (int i = 0; i < numLayers; i++)
	{
		solvers.push_back(new GeometrySolver());
	}
}

MultiGeometrySolver::~MultiGeometrySolver()
{
	for (auto s:solvers)
		delete s;
}

#define CROP_TO_2D_IF_NEEDED(V,d) (V.cols()== 3&&d==2)?V.leftCols<2>():V

const bool USE_SPARSE_WEIGHTS = true;

void MultiGeometrySolver::init(const Eigen::MatrixXd& V, const Eigen::MatrixXi& TF, const std::vector<SubspaceInfo*> infos)
{

	if (TF.cols() != 3 && TF.cols() != 4)
	{
		std::cerr<<"MultiGeometrySolver::Init() fails on TF!"<<std::endl;
		return;
	}
	assert(TF.cols() == 3 || TF.cols() == 4);
	assert(solvers.size() == infos.size());

	subspaceInfos.clear();
	dim = TF.cols() - 1;

	assert(V.cols() == 3 || V.cols() == 2 && dim == 2);

	const Eigen::MatrixXd VV = CROP_TO_2D_IF_NEEDED(V,dim);

	for (size_t i = 0; i < solvers.size(); i++)
	{
		ARAP_para ap;

		ap.var = CROP_TO_2D_IF_NEEDED(infos[i]->var,dim);

		ap.with_subspace_ARAP = true;
		ap.use_sparse_weights = USE_SPARSE_WEIGHTS;

		if (ap.with_subspace_ARAP)
		{
			if (ap.use_sparse_weights)
			{
				ap.subspace_M_sparse = infos[i]->bases_sparse;
			} 
			else
			{
				ap.subspace_M = infos[i]->bases;			
			}
			
			ap.rotation_group = infos[i]->rotation_group;//ARAP_rotation_group;
		}

		ap.dim = dim;

		ap.with_dynamics = false;

		if (dim == 2)
			ap.mesh = ARAP_mesh(VV, Eigen::MatrixXi(0, 3), TF);
		else
			ap.mesh = ARAP_mesh(VV, TF, Eigen::MatrixXi(0, 4) );

		ap.phys_para = ARAP_phys_para(1.,1.,1.);//(time_step, mu, rho);

		// ap.per_group_weight = ARAP_per_group_weight; // it is OK for not settign this one

		solvers[i]->init(ap);

		subspaceInfos.push_back(*infos[i]);
	}

	has_inited = true;
}

void MultiGeometrySolver::update(int update_id)
{
	if (update_id < 0)
		update00();
	else
		update01(update_id);
}

void MultiGeometrySolver::update00()
{
	double gravity[3] = { 0.,0.,0. };
	for (size_t i = 0; i < solvers.size(); i++)//auto s:solvers)
	{
		solvers[i]->update(1, 1, true, gravity, 1.);

		// bypass to the next layer
		if (i<solvers.size()-1)
		{
			if(USE_SPARSE_WEIGHTS)
				solvers[i + 1]->setVar(subspaceInfos[i].bases_bypass_sparse * solvers[i]->getVar());
			else
				solvers[i + 1]->setVar( subspaceInfos[i].bases_bypass * solvers[i]->getVar() );
		}
	}
}

void MultiGeometrySolver::getV(Eigen::MatrixXd& VV) const
{
	getV(NUM_OF_MULTI_GRID_LEVEL, VV);
}

void MultiGeometrySolver::getV(int layer, Eigen::MatrixXd& VV) const
{
	if (layer >= numLayers)
	{
		layer = numLayers - 1;
		std::cout << "Error in getV()" << std::endl;
	}
	if (layer < 0)
	{
		layer = 0;
		std::cout << "Error in getV()" << std::endl;
	}

	VV = solvers[layer]->getV();
}

void MultiGeometrySolver::getV3d(Eigen::MatrixXd& VV) const
{
	getV3d(NUM_OF_MULTI_GRID_LEVEL - 1, VV);
}

void MultiGeometrySolver::getV3d(int layer, Eigen::MatrixXd& VV) const
{
	if (layer >= numLayers)
	{
		layer = numLayers - 1;
		std::cout << "Error in getV()" << std::endl;
	}
	if (layer < 0)
	{
		layer = 0;
		std::cout << "Error in getV()" << std::endl;
	}

	auto tmp = solvers[layer]->getV();
	VV.resize(tmp.rows(),3);
	VV.leftCols(tmp.cols()) = tmp;
}

#include <eigen_helper.h>
void MultiGeometrySolver::setConstraint(const Eigen::VectorXi& known, const Eigen::MatrixXd& knownPos)
{
	assert(knownPos.cols() == 3 || knownPos.cols() == 2 && dim == 2);

	// Preprocessing
	const Eigen::MatrixXd kP = CROP_TO_2D_IF_NEEDED(knownPos,dim);//(knownPos.cols() == 3 && dim == 2) ? knownPos.leftCols<2>() : knownPos;

	if (!has_inited)
	{
		std::cout << "setConstraint(): ignored since not initialized!" << std::endl;
		return;
	}

	for (auto s:solvers)
	{
		// this relies on the fact of how b0, b1, b2 are ordered
		const Eigen::VectorXi I = NaturalSeq(known.rows());//subspaceInfos[0].b.block(0, 0, known.rows(), 1);// 
		s->setConstraints(I, kP);
	}
}

void MultiGeometrySolver::update01(int start_level)
{
	//const int start_level = 0;

	double gravity[3] = { 0.,0.,0. };
	solvers[start_level]->update(1, 1, true, gravity, 1.);
	
	for (size_t i = start_level; i < solvers.size(); i++)//auto s:solvers)
	{
		// bypass to the next layer
		if (i < solvers.size() - 1)
		{
			if (USE_SPARSE_WEIGHTS)
				solvers[i + 1]->setVar(subspaceInfos[i].bases_bypass_sparse * solvers[i]->getVar());
			else
				solvers[i + 1]->setVar(subspaceInfos[i].bases_bypass * solvers[i]->getVar());
		}
	}
}

void MultiGeometrySolver::update02()
{
	const int start_level = 1;

	double gravity[3] = { 0.,0.,0. };
	solvers[start_level]->update(1, 1, true, gravity, 1.);

	for (size_t i = start_level; i < solvers.size(); i++)//auto s:solvers)
	{
		// bypass to the next layer
		if (i < solvers.size() - 1)
		{
			if (USE_SPARSE_WEIGHTS)
				solvers[i + 1]->setVar(subspaceInfos[i].bases_bypass_sparse * solvers[i]->getVar());
			else
				solvers[i + 1]->setVar(subspaceInfos[i].bases_bypass * solvers[i]->getVar());
		}
	}
}

void MultiGeometrySolver::update03()
{
	const int start_level = 2;

	double gravity[3] = { 0.,0.,0. };
	solvers[start_level]->update(1, 1, true, gravity, 1.);

	for (size_t i = start_level; i < solvers.size(); i++)//auto s:solvers)
	{
		// bypass to the next layer
		if (i < solvers.size() - 1)
		{
			if (USE_SPARSE_WEIGHTS)
				solvers[i + 1]->setVar(subspaceInfos[i].bases_bypass_sparse * solvers[i]->getVar());
			else
				solvers[i + 1]->setVar(subspaceInfos[i].bases_bypass * solvers[i]->getVar());
		}
	}
}
//#include <igl/biharmonic_coordinates.h>

/*
there will be two types of
[bt;b0] [bt;b1] [bt;b2] [bt;b3] ... [bt;bN]
[bt;bN] needs to include all vertices of the mesh
*/

#include <farthest_point_sampling.h>
#include <igl/slice.h>
#include <igl/colon.h>
#include <compute_weights.h>
#include <eigen_helper.h>
#include <math.h>
#include <weights_reduction.h>
#include <reproject_coordinate.h>
void generate_subspace_info(const Eigen::MatrixXd& V, const Eigen::MatrixXi& TF, const Eigen::VectorXi& b0, std::vector<SubspaceInfo*>& infos)
{
	const int dim = 2; // TODO change this.

	Eigen::VectorXi b;
	farthest_point_sampling(V, V.rows() - b0.rows(), b0, b, 0);

	std::vector<SubspaceInfo*> si;
	const int numInfos = NUM_OF_MULTI_GRID_LEVEL;

	for (size_t i = 0; i < numInfos; i++)
		si.push_back(new SubspaceInfo);

	//SubspaceInfo* si0 = new SubspaceInfo;
	//SubspaceInfo* si1 = new SubspaceInfo;

	assert(V.rows() >= 80);
	double factor = pow(V.rows() / 80.,1./(NUM_OF_MULTI_GRID_LEVEL-1));

	const int n = V.rows();
	const int tf = TF.rows();

	for (size_t i = 0; i < numInfos - 1; i++)
	{
		si[i]->b = b.block(0, 0, int(80 * pow(factor,i) ), 1);
	}
	si[numInfos-1]->b = b;

	//si[0]->b = b.block(0, 0, 80, 1);
	//si[1]->b = b.block(0, 0, int( 80 * factor), 1);
	//si[2]->b = b;

	for (size_t i = 0; i < numInfos; i++)
		igl::slice(V, si[i]->b, 1, si[i]->var);

	for (size_t i = 0; i < numInfos; i++)
		if(si[i]->b.rows()!=V.rows()) 
			bilaplacian_coordinates(V, TF, si[i]->b, si[i]->bases);

	for (size_t i = 0; i < numInfos; i++)
	{
		if (si[i]->b.rows() == V.rows())
		{
			reorder_matrix(si[i]->b, si[i]->bases_sparse);
			continue;
		}
			
		Eigen::MatrixXd WV;// WV;
		Eigen::MatrixXi WI;// WI;
		auto H = si[i]->var;
		weights_reduction(si[i]->bases, 20, WV, WI);
		reproject_coordinate_individually(WV, WI, V, H, dim, 1.);
		si[i]->bases_sparse = weights_pair_to_weights(WV, WI, si[i]->bases.cols());
	}
			
	for (size_t i = 0; i < numInfos - 1; i++)
		igl::slice(si[i]->bases, si[i+1]->b, 1, si[i]->bases_bypass);

	for (size_t i = 0; i < numInfos - 1; i++)
		igl::slice(si[i]->bases_sparse, si[i + 1]->b, 1, si[i]->bases_bypass_sparse);
	
	bool full_cluster = true;//false;
	if (full_cluster)
	{
		// full cluster
		for (size_t i = 0; i < numInfos; i++)
			si[i]->rotation_group = NaturalSeq(TF.rows());
	} 
	else
	{
		for (size_t i = 0; i < numInfos; i++)
			farthest_element_clustering(V, TF, int( si[i]->b.rows()*tf*1./n + 0.49), si[i]->rotation_group); // choose the number of rotation cluster to be exactly the number of points
	}

	infos = si;
}
