
#include "linear_solver.h"

//extern "C" {
#include <cholmod.h>
//}

typedef SuiteSparse_long UF_long;

#include <Eigen/Cholesky>
#include <Eigen/CholmodSupport>

//bool linear_solver(
//	Eigen::SparseMatrix<double>& A,
//	const Eigen::MatrixXd& B,
//	//const Eigen::VectorXi& known,
//	//const Eigen::MatrixXd& knowY,
//	//const Eigen::SparseMatrix<double>& Aeq,
//	//const Eigen::MatrixXd& Beq,
//	Eigen::MatrixXd& Z
//	)

//template<
//	typename DerivedA,
//	typename DerivedB,
//	typename DerivedZ>
//bool linear_solver(
//	const Eigen::SparseMatrix<DerivedA>& A,
//	const Eigen::PlainObjectBase<DerivedB>& B,
//	Eigen::PlainObjectBase<DerivedZ>& Z
//	)
bool linear_solver(
	const Eigen::SparseMatrix<double>& A,
	const Eigen::MatrixXd& B,
	Eigen::MatrixXd& Z
	)
{
	
	if (false)
	{
		//Eigen::SimplicialLLT<Eigen::SparseMatrix<DerivedA>> llt;
		Eigen::SimplicialLLT<Eigen::SparseMatrix<double>> llt;
		llt.analyzePattern(A);
		llt.factorize(A);
	} 
	else
	{
		//Eigen::CholmodDecomposition<Eigen::SparseMatrix<DerivedA>> cholmod;
		Eigen::CholmodDecomposition<Eigen::SparseMatrix<double>> cholmod;
		//Eigen::CholmodSimplicialLLT<Eigen::SparseMatrix<double>> cholmod;
		cholmod.analyzePattern(A);
		cholmod.factorize(A);
		Z = cholmod.solve(B);
	}
	
//	cholmod.factorize(A);

	

	return true;
}
