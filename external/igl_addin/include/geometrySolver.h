#ifndef GEOMETRY_SOLVER_H
#define GEOMETRY_SOLVER_H

#include <Eigen/Core>
#include <Eigen/Sparse>

//IGL ADDIN
#include <quadSolver.h>

class ARAP_phys_para
{
public:
	ARAP_phys_para(double t = 1., double mu = 1., double rho = 1.) : time_step(t)
	{
		this->mu = mu;
		this->rho = rho;
	}
	double time_step;
	double mu;
	double rho;
};

class ARAP_mesh
{
public:
	ARAP_mesh(){}
	ARAP_mesh(
		const Eigen::MatrixXd& VV,
		const Eigen::MatrixXi& TT,
		const Eigen::MatrixXi& FF
		) :V(VV), T(TT), F(FF){}

	Eigen::MatrixXd V;
	Eigen::MatrixXi T;
	Eigen::MatrixXi F;
};

class ARAP_para
{
public:
	ARAP_para() :
		dim(2),
		with_subspace_ARAP(false),
		with_dynamics(false),
		use_sparse_weights(false)
	{}

	ARAP_para(int d, bool ws, bool wd) :
		dim(d),
		with_subspace_ARAP(ws),
		with_dynamics(wd)
	{}

	Eigen::MatrixXd var; // variables, equal to mesh.V for full arap energy, not equal for subspace arap.

	ARAP_mesh mesh; // the entire mesh used to compute ARAP energy
	ARAP_phys_para phys_para;

	int dim;

	bool with_subspace_ARAP;
	bool use_sparse_weights;

	Eigen::MatrixXd subspace_M;
	Eigen::SparseMatrix<double> subspace_M_sparse;

	Eigen::VectorXi rotation_group;

	bool with_dynamics;

	Eigen::VectorXd per_group_weight;
};

class GeometrySolver{

private:

	Eigen::MatrixXi T;
	Eigen::MatrixXd V0;//Initial vertices positions
	Eigen::MatrixXd V;//Current vertices positions
	
	Eigen::MatrixXd Varibles0;//Initial values of Varibles.
	Eigen::MatrixXd Varibles;//This is what called T in the FAST paper.

	Eigen::MatrixXd Varibles_last;
	Eigen::MatrixXd Varibles_last_last;

	Eigen::MatrixXd Forces;//Internal Elastic Forces

	Eigen::MatrixXd Vel;//Velocity

	Eigen::VectorXi S;//Indication of Constrained Varibles (NOT vertices!!), 1,>=0:constrained, -1 not constrained
	
	QuadMatrixSolver quadSolver;
	

//#define USE_ALEC_STYLE_ARAP_IMPLEMENTATION
#ifdef USE_ALEC_STYLE_ARAP_IMPLEMENTATION
	//Used by alec's implementation
	Eigen::SparseMatrix<double> K_alec;
	Eigen::SparseMatrix<double> CSM;//covariance scatter matrix
#else
	//Used by wangyu style implementation, which exactly follows the FAST paper.
	Eigen::SparseMatrix<double> K_right;
	//Eigen::SparseMatrix<double> K_in_paper;
	Eigen::SparseMatrix<double> K_reduced;
	Eigen::SparseMatrix<double> K_reduced_hat;

	Eigen::VectorXd Scale_Weight_reduced;
#endif

	bool with_subspace_weights;
	Eigen::SparseMatrix<double> M;//weights such that V=W*H, H are handles
	Eigen::VectorXi rotation_group;
	Eigen::VectorXd per_group_weight;
	Eigen::SparseMatrix<double> RGProject; // Rotation Group Projection Matrix.

	// Data for dynamics
	bool with_dynamics;

	double time_step;

	double mu;

	Eigen::SparseMatrix<double> Mass;// The mass matrix used for dymanics simulation.

	Eigen::SparseMatrix<double> Mass_reduced;

	Eigen::MatrixXd ConstantMassM;

private:
	// by now only supports the case VV==VV0
	void set_mesh(const Eigen::MatrixXi& TT, const Eigen::MatrixXd& VV, const Eigen::MatrixXd& VV0);

	// With dynamics, (if with) time step, mu, Mass.
	void precomputation(bool wd, double dt, double mu, const Eigen::SparseMatrix<double>& Mass);// The mass matrix used for dymanics simulation.);

	double phys_update_once(int arap_iters, bool is_arap, bool fakeUpdate, double * gravity, double damping = 1.0);

	double update(int arap_iters, int phys_iter, bool is_arap, bool fakeUpdate, double * gravity, double damping = 1.0);// true for arap, false for asap; fake update means only return energy value, no real update

public:

	GeometrySolver();

	void init(const ARAP_para& para);

	void set_mesh(const Eigen::MatrixXi& TT, const Eigen::MatrixXd& VV){set_mesh(TT, VV, VV);}
	
	void setSubspaceWeights(const Eigen::SparseMatrix<double>& m, const Eigen::VectorXi& rg, const Eigen::MatrixXd& var0);//this should be called just before precomputation if setting weights 

	void precomputation_no_dynamics();

	void precomputation_with_dynamics(double dt, double mu, double rho);// The mass matrix used for dymanics simulation.);

	const Eigen::MatrixXd getV() const;

	const Eigen::MatrixXd& getVar() const { return Varibles; }

	void setVar(const Eigen::MatrixXd& var);

	double update(int arap_iters, int phys_iter, bool is_arap, double * gravity, double damping = 1.0)//real update
	{
		return update(arap_iters, phys_iter, is_arap, false, gravity, damping);
	}

	double fakeUpdate(bool is_arap, const Eigen::MatrixXd& new_Var)// true for arap, false for asap; fake update means only return energy value, no real update
	{
		Varibles = new_Var;
		double gravity[3] = { 0. };
		return update(1, is_arap, true, gravity);
		// it does not make sense to have more than 1 iterations
	}

	void setConstraints(const Eigen::VectorXi& known, const Eigen::MatrixXd& knownPos, const Eigen::MatrixXd& Meq, const Eigen::MatrixXd& Peq);

	void setConstraints(const Eigen::VectorXi& known, const Eigen::MatrixXd& knownPos);

	void setConstraintsIndicator(const Eigen::VectorXi& SS, const Eigen::MatrixXd& Var_new, const Eigen::MatrixXd& Meq, const Eigen::MatrixXd& Peq);
	
	void setPerGroupWeight(const Eigen::VectorXd& gw);

	//only V_new with >= 0 value will be used
	bool need_restart_when_constrant_change();
	//utility function
	//int num_in_selection(const Eigen::VectorXi & S);

	void perElementEnergy(Eigen::VectorXd& energy);

	void perGroupEnergy(Eigen::VectorXd& energy);
};


#endif GEOMETRY_SOLVER_H