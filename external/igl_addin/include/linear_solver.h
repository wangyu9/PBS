

#include <Eigen/Dense>
#include <Eigen/Sparse>


//template<
//	typename DerivedA,
//	typename DerivedB,
//	typename DerivedZ>
//	bool linear_solver(
//	const Eigen::SparseMatrix<DerivedA>& A,
//	const Eigen::PlainObjectBase<DerivedB>& B,
//	Eigen::PlainObjectBase<DerivedZ>& Z
//	);

bool linear_solver(
	const Eigen::SparseMatrix<double>& A,
	const Eigen::MatrixXd& B,
	Eigen::MatrixXd& Z
	);