#include "quadSolver.h"

#ifdef USING_IGL_HEADER_ONLY_MODE
#define IGL_HEADER_ONLY 
#endif



// IGL ADDIN
#include "quadProg.h"

int num_in_selection(const Eigen::VectorXi & S)
{
	int count = 0;
	for(int v = 0;v<S.rows(); v++)
	{
		if(S(v) >= 0)
		{
			count++;
		}
	}
	return count;
}

void QuadSolverBase::precompute(const Eigen::SparseMatrix<double>& new_Q)
{
	Q = new_Q;

	rePrecompute();
}

void QuadSolverBase::rePrecompute()
{
	if(Q.rows()==0)
	{
		printf("Error Q is not set yet, must call precompute first!");
		return;
	}
	// redo quad precompute
	Eigen::SparseMatrix<double> Aeq_sparseview = Aeq.sparseView();
	if (precomp_data!=NULL)
	{
		delete precomp_data;
	}
	precomp_data = new igl::min_quad_with_fixed_data < double > ;
	igl::min_quad_with_fixed_precompute( 
		Q,known,Aeq_sparseview,true,*precomp_data);
}

void QuadSolverBase::setKnownIndexOnly(const Eigen::VectorXi& S)
{
	// get b from S
	int num_known = num_in_selection(S);
	Eigen::VectorXi temp_known(num_known);


	{
		int bi = 0;
		for(int v = 0;v<S.rows(); v++)
		{
			if(S(v) >= 0)
			{
				temp_known(bi) = v;
				bi++;
			}
		}
	}

	bool known_unchanged = (temp_known.rows()==known.rows()) 
		&& ( (temp_known.array()==known.array()).all() );

	if (!known_unchanged)
	{
		known = temp_known;
		rePrecompute();
	}

	//This is INCORRECT!!// get b from S
	//b.resize(num_in_selection(S));
	//bc = Eigen::MatrixXd::Zero(b.size(),S.maxCoeff()+1);
	//{
	//	int bi = 0;
	//	for(int v = 0;v<S.rows(); v++)
	//	{
	//		if(S(v) >= 0)
	//		{
	//			b(bi) = v;
	//			bc(bi,S(v)) = 1;
	//			bi++;
	//		}
	//	}
	//}
}

void QuadSolverBase::setKnownAndAeq(const Eigen::VectorXi& S, const Eigen::MatrixXd& new_Aeq)
{
	// get b from S
	int num_known = num_in_selection(S);
	Eigen::VectorXi temp_known(num_known);


	{
		int bi = 0;
		for(int v = 0;v<S.rows(); v++)
		{
			if(S(v) >= 0)
			{
				temp_known(bi) = v;
				bi++;
			}
		}
	}

	bool known_unchanged = (temp_known.rows()==known.rows()) 
		&& ( temp_known.rows()==0||(temp_known.array()==known.array()).all() );

	bool Aeq_unchanged = (Aeq.rows()==new_Aeq.rows())
		&& (Aeq.cols() == new_Aeq.cols())
		&& ( Aeq.rows()==0||(Aeq.array()==new_Aeq.array()).all() );

	known = temp_known;
	Aeq = new_Aeq;

	if (!known_unchanged || !Aeq_unchanged)
	{
		printf("Quad Solver Base setKnownAndAeq(): left hand side changed, recompute!\n");
		rePrecompute();
	}

	//This is INCORRECT!!// get b from S
	//b.resize(num_in_selection(S));
	//bc = Eigen::MatrixXd::Zero(b.size(),S.maxCoeff()+1);
	//{
	//	int bi = 0;
	//	for(int v = 0;v<S.rows(); v++)
	//	{
	//		if(S(v) >= 0)
	//		{
	//			b(bi) = v;
	//			bc(bi,S(v)) = 1;
	//			bi++;
	//		}
	//	}
	//}
}

//#include <print_matlab.h>
double QuadSolverBase::solve(const Eigen::VectorXd& B, const Eigen::VectorXd& knownY, const Eigen::VectorXd& Beq, Eigen::VectorXd& Z)
{
	//print_matlab(Q.toDense(),"Q");
	//print_matlab(B.cast<double>(),"B");
	//print_matlab(Aeq,"Aeq");
	//print_matlab(Beq.cast<double>(),"Beq");
	//print_matlab(known.cast<double>(),"known");
	//print_matlab(knownY.cast<double>(),"knownY");


	igl::min_quad_with_fixed_solve(*precomp_data,
		B,
		knownY,
		Beq,
		Z);

	return fakeSolve(B, Z);

	//Eigen::MatrixXd ZZ;
	//const Eigen::MatrixXd QQ = Q.toDense();
	//const Eigen::MatrixXd BB = B;
	//const Eigen::MatrixXd AAeq = Aeq;
	//const Eigen::MatrixXd BBeq = Beq;
	//quadprog(QQ,BB,AAeq,BBeq,ZZ);
	//Z = ZZ.col(0);
}

double QuadSolverBase::fakeSolve(const Eigen::VectorXd& B, const Eigen::VectorXd& Z)
{

	return 0.5*(Z.transpose()*Q*Z)(0,0) + (Z.transpose()*B)(0,0);// though the result is a 1 by 1 matrix.
	
	//Eigen::MatrixXd ZZ;
	//const Eigen::MatrixXd QQ = Q.toDense();
	//const Eigen::MatrixXd BB = B;
	//const Eigen::MatrixXd AAeq = Aeq;
	//const Eigen::MatrixXd BBeq = Beq;
	//quadprog(QQ,BB,AAeq,BBeq,ZZ);
	//Z = ZZ.col(0);
}

void QuadSolver::setKnownValueOnly(const Eigen::VectorXi& S, Eigen::VectorXd Y)
{
	//setKnownValueOnly

	// get b from S
	knownY.resize(num_in_selection(S), Y.cols());
	{
		int bi = 0;
		for(int v = 0;v<S.rows(); v++)
		{
			if(S(v) >= 0)
			{
				knownY.row(bi) = Y.row(v);
				bi++;
			}
		}
	}
}

void QuadMatrixSolver::setKnownValueOnly(const Eigen::VectorXi& S, Eigen::MatrixXd Y)
{
	//setKnownValueOnly

	// get b from S
	knownY.resize(num_in_selection(S), Y.cols());
	{
		int bi = 0;
		for(int v = 0;v<S.rows(); v++)
		{
			if(S(v) >= 0)
			{
				knownY.row(bi) = Y.row(v);
				bi++;
			}
		}
	}
}
