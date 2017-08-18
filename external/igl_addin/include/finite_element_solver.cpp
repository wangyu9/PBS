#include "finite_element_solver.h"
#include <iostream>


bool EigenSolverLLT::analyzePattern(const Eigen::SparseMatrix<double>& A)
{
	llt.analyzePattern(A);
	return llt.info() == Eigen::Success;
}

bool EigenSolverLLT::factorize(const Eigen::SparseMatrix<double>& A)
{
	llt.factorize(A);
	return llt.info() == Eigen::Success;
}

bool EigenSolverLLT::solve(const Eigen::MatrixXd& b, Eigen::MatrixXd& x)
{
	x = llt.solve(b);
	return llt.info() == Eigen::Success;
}

FiniteElementSolverBase::FiniteElementSolverBase(): dim(0), has_init(false)
{
	
}

#define CROP_TO_2D_IF_NEEDED(V,d) (V.cols()== 3&&d==2)?V.leftCols<2>():V

bool FiniteElementSolverBase::init(const Eigen::MatrixXd& V, const Eigen::MatrixXi& TF)
{
	using namespace Eigen;

	dim = TF.cols() - 1;

	if (dim!=2&&dim!=3)
	{
		std::cerr << "Error: FiniteElementSolverBase(): Unsuppported dimension!" << std::endl;
		return false;
	}

	if (!(dim == V.cols() || dim == 2 && V.cols() == 3))
	{
		std::cerr << "Error: FiniteElementSolverBase(): Incorrect dimension!" << std::endl;
		return false;
	}

	this->V = CROP_TO_2D_IF_NEEDED(V,dim);
	this->TF = TF;

	V0 = this->V;

	const int n = V.rows();
	Var0 = MatrixXd::Zero(n * dim, 1);
	for (int c = 0; c < dim; c++)
	{
		Var0.block(n*c, 0, n, 1) = V0.block(0, c, n, 1);
	}
	//another way to do so//Var0.resize(Var0.size(),1);//convert to a column vector.

	Var = Var0;

	Var_last_last = Var_last = Var;

	should_refactor = true;

	has_init = true;

	return true;
}

#include <eigen_helper.h>
#include <igl/slice.h>
#include <igl/cat.h>
#include <igl/slice_into.h>
#include <igl/colon.h>
void FiniteElementSolverBase::update()
{
	using namespace Eigen;

	const int n = V.rows();

	// enforce constraints
	for (int i = 0; i < known.rows(); i++)
	{
		for (int c = 0; c < dim; c++)
		{
			Var(c*n + known(i)) = knownPos(i, c);
		}
	}

	SparseMatrix<double> hessian;
	MatrixXd gradient;

	compute_hessian(hessian);
	compute_gradient(gradient);

	gradient =  gradient;
	hessian = -hessian;

	//gradient = hessian * (Var-Var0);

	MatrixXd x;

	SparseMatrix<double> Auu, Auv;

	//VectorXi knownVar = igl::cat(1, igl::cat(1, known, VectorXi(known.array() + n)), VectorXi(known.array()+2*n));
	//VectorXi unknownVar = igl::cat(1, igl::cat(1, unknown, VectorXi(unknown.array() + n)), VectorXi(unknown.array() + 2 * n));

	Eigen::VectorXi unknown;
	Eigen::VectorXi knownVar, unknownVar;

	unknown = complementary(known, n);

	for (int i = 0; i < dim; i++)
	{
		knownVar = (igl::cat(1, knownVar, VectorXi(known.array() + i*n))).eval();
		unknownVar = (igl::cat(1, unknownVar, VectorXi(unknown.array() + i*n))).eval();
	}

	igl::slice(hessian, unknownVar, unknownVar, Auu);
	igl::slice(hessian, unknownVar, knownVar, Auv);

	MatrixXd bu,bv;
	igl::slice(Var, unknownVar, bu);
	igl::slice(Var, knownVar, bv);
	
	// Auu * bu + Auv * bv

	MatrixXd gu, gv;
	igl::slice(gradient, unknownVar, gu);
	igl::slice(gradient, knownVar, gv);


	if (should_refactor)
	{
		linearSolver.analyzePattern(Auu);
		linearSolver.factorize(Auu);
		should_refactor = false;
	}
		
	// gradient + A(x1-x0) = 0
	// x1 = x0 - A\gradient;
	// linearSolver.factorize(hessian);
	// linearSolver.solve(-gradient, x);

	// gu + Auu*(bu1-bu0) + Auv * (bv-bv) = 0
	// bu1 = bu0 - A\(gu) // note there is no contribution from Auv*bv
	

	MatrixXd tmp = -gu;// -Auv*bv;
	linearSolver.solve(tmp, x);

	MatrixXd dVar = MatrixXd::Zero(Var.rows(),Var.cols());
	igl::slice_into(x, unknownVar, 1, dVar);

	Var = Var + dVar;

	// convert Var to V	
	for (int i = 0; i < dim; i++)
	{
		//V.col(i) = Var.block(i*n, 0, n, 1);
		V.block(0, i, n, 1) = Var.block(i*n, 0, n, 1);
	}
}

void FiniteElementSolverBase::getV3d(Eigen::MatrixXd& VV)
{
	VV = Eigen::MatrixXd::Zero(V.rows(), 3);
	VV.leftCols(V.cols()) = V;
}

void FiniteElementSolverBase::setConstraint(const Eigen::VectorXi& known, const Eigen::MatrixXd& knownPos)
{
	using namespace Eigen;

	assert(knownPos.cols() == 3 || knownPos.cols() == 2 && dim == 2);

	// Preprocessing
	const MatrixXd kP = CROP_TO_2D_IF_NEEDED(knownPos, dim);//(knownPos.cols() == 3 && dim == 2) ? knownPos.leftCols<2>() : knownPos;

	const bool known_changed = !(
		(this->known.size() == known.size())
		&& (this->known.array()==known.array()).all() );

	should_refactor = should_refactor || known_changed;

	this->known = known;
	this->knownPos = kP;

}

//void assemble_gradient(const int n, const int dim, const Eigen::MatrixXi& TF, const Eigen::MatrixXd& gradient_per_element, Eigen::MatrixXd& gradient)
//{
//
//}

void FiniteElementSolverBase::compute_gradient(Eigen::MatrixXd& gradient) const
{
	using namespace Eigen;

	const int n = V.rows();
	gradient = MatrixXd::Zero(n*dim,1);

	// looping over all elements
	for (int tf = 0; tf < TF.rows(); tf++)
	{
		// preparing elements
		MatrixXd ele(dim + 1, dim);
		MatrixXd ele0(dim + 1, dim);
		for (int r = 0; r < dim + 1; r++)
		{
			ele.row(r) = V.row(TF(tf,r));
			ele0.row(r) = V0.row(TF(tf,r));
		}

		// compute gradients for each of the element
		MatrixXd gradient_per_element;
		compute_gradient_per_element(ele0, ele, gradient_per_element);

		// assemble gradient per element to the total gradient
		for (int r = 0; r < dim + 1; r++)
		{
			for (int c = 0; c < dim; c++)
			{
				const int index_total = c*n + TF(tf, r);
				const int index_ele = c*(dim + 1) + r;
				gradient(index_total) = gradient_per_element(index_ele);
			}
		}

	}
}

template <typename T>
inline void blkdiag(
	const Eigen::SparseMatrix<T> & A,
	const int n,
	Eigen::SparseMatrix<T> & B)
{
	assert(n > 0);
	B = Eigen::SparseMatrix<T>(n*A.rows(), n*A.cols());//.resize(n*A.rows(), n*A.cols());
	B.reserve(n*A.nonZeros());
	for (int i = 0; i < n; i++)
	{
			// Loop outer level
			for (int k = 0; k < A.outerSize(); ++k)
			{
				// loop inner level
				for (typename Eigen::SparseMatrix<T>::InnerIterator it(A, k); it; ++it)
				{
					B.insert(i*A.rows() + it.row(), i*A.cols() + it.col()) = it.value();
				}
			}
	}
	B.finalize();
}

#include <igl/cotmatrix.h>
void FiniteElementSolverBase::compute_hessian(Eigen::SparseMatrix<double>& hessian) const
{
	using namespace Eigen;

	const int n = V0.rows();

	Eigen::SparseMatrix<double> laplacian;
	igl::cotmatrix(V0, TF, laplacian);

	blkdiag(laplacian, dim, hessian);

	//hessian = SparseMatrix<double>(n*dim, n*dim);
	//hessian.setIdentity();

}

void displacement_matrix(const Eigen::MatrixXd& ele, Eigen::MatrixXd& D)
{
	using namespace Eigen;
	D.resize(ele.cols(), ele.cols());//= ele0.block(0,0,ele.cols(),ele.cols());
	for (int i = 0; i < ele.cols(); i++)
	{
		D.row(i) = ele.row(i) - ele.row(ele.cols());
	}
	D.transposeInPlace();
}

// this one computes the gradient ?E/?(x,y,z) 
// the entry is organized in the vector (dim+1)*dim by 1 [ ?E/?x1,?E/?x2,?E/?x3,?E/?x4; ?E/?y1,?E/?y2,?E/?y,?E/?y4; ?E/?z1,?E/?z2,?E/?z3,?E/?z4 ]^T 
void FiniteElementSolverBase::compute_gradient_per_element(const Eigen::MatrixXd& ele0, const Eigen::MatrixXd& ele, Eigen::MatrixXd& gradient_per_element) const
{
	compute_gradient_per_element_in_matrix_from(ele0,ele,gradient_per_element);
	// convert to vector
	gradient_per_element.resize(gradient_per_element.size(),1);
}

//#include <igl/svd3x3.h>
//inline void svd(const Eigen::MatrixXd& F, Eigen::MatrixXd& U, Eigen::MatrixXd& S, Eigen::MatrixXd& V)
//{
//	using namespace Eigen;
//
//	if (F.rows() == 3)
//	{
//		Matrix<double, 3, 3> FF, UU, VV;
//		Matrix<double, 3, 1> SS;
//
//		FF = F;
//
//		igl::svd3x3(FF, UU, SS, VV);
//	
//		U = UU;
//		S = SS;
//		V = VV;
//	}
//	else
//	{
//		;// igl::
//	}
//}

#include <igl/polar_svd3x3.h>
#include <igl/polar_svd.h>
inline void polar_decomposition(const Eigen::MatrixXd& F, Eigen::MatrixXd& R)
{
	using namespace Eigen;

	if (F.rows() == 3)
	{
		Matrix<double, 3, 3> FF, RR;
		FF = F;

		igl::polar_svd3x3(FF, RR);

		R = RR;
	}
	else
	{
		Matrix<double, 2, 2> FF, RR, TT, UU, VV;
		Matrix<double, 2, 1> vec;
		FF = F;

		igl::polar_svd(FF, RR, TT, UU, vec, VV);

		R = RR;
	}
}

Eigen::MatrixXd FiniteElementSolverBase::piola_kirchhoff_stress_tensor(const Eigen::MatrixXd& F) const 
{
	using namespace Eigen;

	// P_F is Piola-Kirchhoff stress tensor
	MatrixXd P_F;

	const MatrixXd I = MatrixXd::Identity(dim, dim);

	P_F = 2*(F-I);
	//P_F = ;

	MatrixXd R;
	polar_decomposition(F, R);
	P_F = 2 * (F - R);
	
	return P_F;
}

// this one computes the gradient ?E/?(x,y,z) 
// the entry is organized in the matrix dim+1 by dim 
// [ ?E/?x1,?E/?x2,?E/?x3,?E/?x4; 
//	 ?E/?y1,?E/?y2,?E/?y3,?E/?y4; 
//	 ?E/?z1,?E/?z2,?E/?z3,?E/?z4; ]^T 
void FiniteElementSolverBase::compute_gradient_per_element_in_matrix_from(const Eigen::MatrixXd& ele0, const Eigen::MatrixXd& ele, Eigen::MatrixXd& gradient_per_element) const
{
	using namespace Eigen;

	MatrixXd Ds, Dm;

	displacement_matrix(ele0, Dm);
	displacement_matrix(ele, Ds);

	MatrixXd F = Ds * Dm.inverse();

	double W = abs(Dm.determinant()) / 6.;

	//MatrixXd gradient_F_per_element;
	//compute_gradient_per_element(F, gradient_F_per_element);

	// there are (dim+1) elements each have dim variables (x,y) or (x,y,z).
	gradient_per_element = MatrixXd::Zero(dim+1,dim);

	// assermble gradient_per_element from gradient_F_per_element;

	MatrixXd P_F = piola_kirchhoff_stress_tensor(F);

	MatrixXd minus_H = W * P_F * (Dm.transpose()).inverse();

	for (int i = 0; i < dim; i++)
	{
		gradient_per_element.row(i) = minus_H.col(i).transpose();
		gradient_per_element.row(dim) -= minus_H.col(i).transpose();
	}
}

//void GeometricElasticSolver::compute_gradient_per_element(const Eigen::MatrixXd& F, Eigen::MatrixXd& gradient) const
//{
//	
//} 