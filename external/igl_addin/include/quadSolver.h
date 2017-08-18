#ifndef IGL_ADDIN_QUADSOLVER
#define IGL_ADDIN_QUADSOLVER

#include <Eigen/Core>
#include <Eigen/Sparse>

#include <igl/min_quad_with_fixed.h>

class QuadSolverBase{
public:

	igl::min_quad_with_fixed_data<double>* precomp_data;

	// Quadratic optimization data.

	Eigen::SparseMatrix<double> Q;// This is the quadratic part of energy.
	Eigen::MatrixXd Aeq;
	//Eigen::MatrixXd C;//Constrained Vertex Coordinates
	Eigen::VectorXi known;//Indices of all Constrained Varibles (NOT vertices!!)
	// It makes sense to make known to be a vector for both QuadVectorSolver and QuadMatrixSolver.
	//Eigen::MatrixXd bc;

	QuadSolverBase()
	{
		precomp_data = NULL;
		known.resize(0);
	}

	void Init()
	{
		Q.resize(0,0);
	}

	void precompute(const Eigen::SparseMatrix<double>& new_Q);

	void rePrecompute();

	void setKnownIndexOnly(const Eigen::VectorXi& S);

	void setKnownAndAeq(const Eigen::VectorXi& S, const Eigen::MatrixXd& new_Aeq);

	double solve(const Eigen::VectorXd& B, const Eigen::VectorXd& knownY, const Eigen::VectorXd& Beq, Eigen::VectorXd& Z);

	double fakeSolve(const Eigen::VectorXd& B, const Eigen::VectorXd& Z);
};

class QuadSolver:public QuadSolverBase
{

public:
	Eigen::VectorXd knownY;
	Eigen::VectorXd B;
	Eigen::VectorXd Beq;

	QuadSolver(){
		QuadSolverBase();
	}

	void Init()
	{
		QuadSolverBase::Init();
	}

	void precompute(const Eigen::SparseMatrix<double>& new_Q)
	{
		QuadSolverBase::precompute(new_Q);
	}

	void setKnownValueOnly(const Eigen::VectorXi& S, Eigen::VectorXd Y);
	void setKnownIndexOnly(const Eigen::VectorXi& S)
	{
		QuadSolverBase::setKnownIndexOnly(S);
	}

	void setKnownValueIndex(const Eigen::VectorXi& S, Eigen::VectorXd Y)
	{
		QuadSolver::setKnownIndexOnly(S);
		QuadSolver::setKnownValueOnly(S,Y);
	}

	void setAeqKnownAndY(const Eigen::VectorXi& S, Eigen::VectorXd Y, const Eigen::MatrixXd& new_Aeq)
	{
		QuadSolver::setKnownValueOnly(S,Y);
		QuadSolverBase::setKnownAndAeq(S,new_Aeq);
	}

	void solve(const Eigen::VectorXd& new_B, const Eigen::VectorXd& new_knownY, const Eigen::VectorXd& new_Beq, Eigen::VectorXd& Z)
	{
		B = new_B;
		//knownY = new_knownY;
		Beq = new_Beq;

		QuadSolverBase::solve(B, new_knownY, Beq, Z);
	}

	void solve(Eigen::VectorXd& Z)
	{
		QuadSolverBase::solve(B, knownY, Beq, Z);
	}

	double fakeSolve(const Eigen::VectorXd& new_B, const Eigen::VectorXd& Z)
	{
		return QuadSolverBase::fakeSolve(B, Z);
	}
};


// This will solve for a quadratic problem of the form Eq. (12) in the [Jacobson et. al 2012]
// It solves the quad min problem in form:
// 0.5*tr(T'AT)+tr(T'*B)
// s.t. Aeq*T=Beq, T(known,:)=knownY
// In this case, each column of T could be optimized individually
// 0.5*t'At+t'*B(:,c)
// s.t. Aeq*t=Beq(:,c), t(known)=knownY(:,c)
// where t=T(:,c)
class QuadMatrixSolver:public QuadSolverBase
{

public:
	Eigen::MatrixXd knownY;
	Eigen::MatrixXd B;
	Eigen::MatrixXd Beq;

	QuadMatrixSolver(){
		QuadSolverBase();
	}

	void Init()
	{
		QuadSolverBase::Init();
	}

	void precompute(const Eigen::SparseMatrix<double>& new_Q)
	{
		QuadSolverBase::precompute(new_Q);
	}

	void setKnownValueOnly(const Eigen::VectorXi& S, Eigen::MatrixXd Y);
	void setKnownIndexOnly(const Eigen::VectorXi& S)
	{
		QuadSolverBase::setKnownIndexOnly(S);
	}

	void setKnownValueIndex(const Eigen::VectorXi& S, Eigen::MatrixXd Y)
	{
		QuadMatrixSolver::setKnownIndexOnly(S);
		QuadMatrixSolver::setKnownValueOnly(S,Y);
	}

	void setAeqKnownAndY(const Eigen::VectorXi& S, const Eigen::MatrixXd& Y, const Eigen::MatrixXd& new_Aeq, const Eigen::MatrixXd& new_Beq )
	{
		Beq = new_Beq;
		QuadMatrixSolver::setKnownValueOnly(S,Y);
		QuadSolverBase::setKnownAndAeq(S,new_Aeq);
	}

	// Assume knownY,B,Beq have been set
	double solve(Eigen::MatrixXd& Z)
	{
		int n = B.rows();
		int m = B.cols();

		if(Beq.rows()==0)
		{
			Beq.resize(0,m);
		}
		if(knownY.rows()==0)
		{
			knownY.resize(0,m);
		}

		assert( n==Q.rows() );
		assert( m==Beq.cols() );
		assert( m==knownY.cols() );

		Z.resize(n,m);
		double optimum = 0;
		for (int c=0; c<m; c++)
		{
			Eigen::VectorXd Z_c;
			optimum += QuadSolverBase::solve(B.col(c), knownY.col(c), Beq.col(c), Z_c);
			assert(Z_c.rows()==Z.rows());
			Z.col(c) = Z_c;
		}
		return optimum;
	}

	double fakeSolve(const Eigen::MatrixXd& new_B, const Eigen::MatrixXd& Z)
	{
		B = new_B;
		return fakeSolve(Z);
	}

	// Interface is similar to solve, but not really solve, only return energy
	double fakeSolve(const Eigen::MatrixXd& Z)
	{
		int n = B.rows();
		int m = B.cols();

		assert( n==Q.rows() );

		double optimum = 0;
		for (int c=0; c<m; c++)
		{
			const Eigen::VectorXd Z_c = Z.col(c);
			optimum += QuadSolverBase::fakeSolve(B.col(c), Z_c);
		}
		return optimum;
	}


	double solve(const Eigen::MatrixXd& new_B, Eigen::MatrixXd& Z)
	{
		B = new_B;
		return solve(Z);
	}

	double solve(const Eigen::MatrixXd& new_B, const Eigen::MatrixXd& new_Beq, Eigen::MatrixXd& Z)
	{
		B = new_B;
		Beq = new_Beq;

		return solve(Z);
	}

	double solve(const Eigen::MatrixXd& new_B, const Eigen::MatrixXd new_knownY, const Eigen::MatrixXd& new_Beq, Eigen::MatrixXd& Z)
	{
		B = new_B;
		knownY = new_knownY;
		Beq = new_Beq;

		return solve(Z);
	}
};


#endif /*IGL_ADDIN_QUADSOLVER*/