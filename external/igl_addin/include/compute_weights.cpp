#include "compute_weights.h"

#include <igl/cotmatrix.h>
#include <igl/massmatrix.h>
//#include <igl/invert_diag.h>
////#include <igl/speye.h>
////#include <igl/slice_into.h>
#include <igl/min_quad_with_fixed.h>
//

#ifdef IGL_STATIC_LIBRARY
#undef IGL_STATIC_LIBRARY
#include <igl/normal_derivative.h>// I faced with a wired bug of template deducing... so use header mode to avoid it.
#include <igl/on_boundary.h>
#define IGL_STATIC_LIBRARY
#endif



//#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET
#include <Eigen/Sparse>

//#include <iostream>
//#include <cstdio>

#include <linear_solver.h>

#include <eigen_helper.h>
#include <igl/slice_into.h>

template<
	typename DerivedV,
	typename DerivedT,
	typename Derivedb,
	typename DerivedW >
	bool bilaplacian_coordinates(
		const Eigen::PlainObjectBase<DerivedV>& V,
		const Eigen::PlainObjectBase<DerivedT>& T,
		const Eigen::PlainObjectBase<Derivedb>& b,
		Eigen::PlainObjectBase<DerivedW>& W)
{
	std::vector<std::vector<int>> S;
	for (int i = 0; i < b.rows(); i++)
	{
		std::vector<int> Si;
		Si.push_back(b(i, 0));
		S.push_back(Si);
	}

	return bilaplacian_coordinates(V, T, S, W);
}


template<
	typename DerivedV,
	typename DerivedT,
	typename SType,
	typename DerivedW >
bool bilaplacian_coordinates(
	 const Eigen::PlainObjectBase<DerivedV>& V,
	 const Eigen::PlainObjectBase<DerivedT>& T,
	 const std::vector<std::vector<SType> > & S,
	 Eigen::PlainObjectBase<DerivedW>& W)
{
	assert(T.maxCoeff() < V.rows());
	std::cout << T.maxCoeff() << std::endl;
	using namespace Eigen;
	using namespace std;
	using namespace igl;//wangyu
	// This is not the most efficient way to build A, but follows "Linear
	// Subspace Design for Real-Time Shape Deformation" [Wang et al. 2015]. 
	SparseMatrix<double> A;
	{
		SparseMatrix<double> N, Z, L, K, M;
#if 1
		//ndef DEBUG 
		//wangyu disable it now
		normal_derivative(V, T, N);
		Array<bool, Dynamic, 1> I;
		Array<bool, Dynamic, Dynamic> C;
		on_boundary(T, I, C);
		{
			std::vector<Triplet<double> >ZIJV;
			for (int t = 0; t < T.rows(); t++)
			{
				for (int f = 0; f < T.cols(); f++)
				{
					if (C(t, f))
					{
						const int i = t + f*T.rows();
						for (int c = 1; c < T.cols(); c++)
						{
							ZIJV.emplace_back(T(t, (f + c) % T.cols()), i, 1);
						}
					}
				}
			}
			Z.resize(V.rows(), N.rows());
			Z.setFromTriplets(ZIJV.begin(), ZIJV.end());
			N = (Z*N).eval();
		}
		cotmatrix(V, T, L);
		K = N + L;
#else
		cotmatrix(V, T, L);
		K = L;
#endif
		massmatrix(V, T, MASSMATRIX_TYPE_DEFAULT, M);
		DiagonalMatrix<double, Dynamic> Minv =
			((VectorXd)M.diagonal().array().inverse()).asDiagonal();
		A = K.transpose() * (Minv * K);
	}
	// Vertices in point handles
	const size_t mp =
		count_if(S.begin(), S.end(), [](const vector<int> & h){return h.size() == 1; });
	// number of region handles
	const size_t r = S.size() - mp;
	// Vertices in region handles
	size_t mr = 0;
	for (const auto & h : S)
	{
		if (h.size() > 1)
		{
			mr += h.size();
		}
	}
	const size_t dim = T.cols() - 1;
	// Might as well be dense... I think...

	const size_t m = mp + r*(dim + 1);// wangyu

	MatrixXd J = MatrixXd::Zero(mp + mr, m);
	Eigen::VectorXi b(mp + mr); //wangyu //VectorXi b(mp+mr);
	MatrixXd H(mp + r*(dim + 1), dim);
	{
		int v = 0;
		int c = 0;
		for (int h = 0; h < S.size(); h++)
		{
			if (S[h].size() == 1)
			{
				H.row(c) = V.block(S[h][0], 0, 1, dim);
				J(v, c++) = 1;
				b(v) = S[h][0];
				v++;
			}
			else
			{
				assert(S[h].size() >= dim + 1);
				for (int p = 0; p < S[h].size(); p++)
				{
					for (int d = 0; d < dim; d++)
					{
						J(v, c + d) = V(S[h][p], d);
					}
					J(v, c + dim) = 1;
					b(v) = S[h][p];
					v++;
				}
				H.block(c, 0, dim + 1, dim).setIdentity();
				c += dim + 1;
			}
		}
	}
	// minimize    ½ W' A W' 
	// subject to  W(b,:) = J
	int n = V.rows();
	Eigen::MatrixXd B = Eigen::MatrixXd::Zero(A.rows(),1);// mp + r*(dim + 1));
	Eigen::VectorXi known = b;
	Eigen::MatrixXd knowY = J;
	Eigen::SparseMatrix<double> Aeq;
	Eigen::MatrixXd Beq = Eigen::MatrixXd();

	//return min_quad_with_fixed(
	//	A, B, known, knowY, Aeq, Beq, true, W);//wangyu VectorXd

	//return min_quad_with_fixed(
	//	A, Eigen::VectorXd::Zero(A.rows()).eval(), b, J, {}, Eigen::VectorXd(), true, W);//wangyu VectorXd

	min_quad_with_fixed_data<double> precomp_data;

	bool result = min_quad_with_fixed_precompute(
		A, known, Aeq, true, precomp_data);
	result = result &&
		min_quad_with_fixed_solve(precomp_data, B, knowY, Beq, W);

	return result;

	SparseMatrix<double> Auu, Auv;
	const Eigen::VectorXi unknown = complementary(known, n);
	//const size_t f = unknown.size();

	igl::slice(A, unknown, unknown, Auu);
	igl::slice(A, unknown, known, Auv);

	Eigen::MatrixXd Wu;
	const Eigen::MatrixXd M = Eigen::MatrixXd::Identity(m,m);
	Eigen::MatrixXd Bu = Auv*M;

	linear_solver(Auu, -Bu, Wu);

	W.resize(n,m);

	W.setConstant(1);// set W known parts
	
	igl::slice_into(Wu, unknown, Eigen::VectorXi::LinSpaced(Sequential, m, 0, m-1), W);// W(uknown,:) = Wu;
	igl::slice_into(M, known, Eigen::VectorXi::LinSpaced(Sequential, m, 0, m - 1), W);// W(known,:) = M;

//	Eigen::CholmodSimplicialLDLT<double> cholmod;
	//cholmod.compute(A);
	//cholmod.solve(B);

	return r;
}


//template<
//		 typename DerivedV,
//		 typename DerivedTF,
//		 typename SType,
//		 typename DerivedW >
//bool bilaplacian_coordinates(
//		 const Eigen::PlainObjectBase<DerivedV>& V,
//		 const Eigen::PlainObjectBase<DerivedTF>& TF,
//		 const std::vector<std::vector<SType> > & S,
//		 Eigen::PlainObjectBase<DerivedW>& W)
//{
//	using namespace igl;
//	using namespace std;
//	using namespace Eigen;
//
//	// number of domain vertices
//	int n = V.rows();
//
//	// number of handles
//	//int m = bc.cols();
//
//	SparseMatrix<typename DerivedW::Scalar> L;
//	cotmatrix(V, TF, L);
//	MassMatrixType mmtype = MASSMATRIX_TYPE_VORONOI;
//	if (TF.cols() == 4)
//	{
//		mmtype = MASSMATRIX_TYPE_BARYCENTRIC;
//	}
//	SparseMatrix<typename DerivedW::Scalar> M;
//	SparseMatrix<typename DerivedW::Scalar> Mi;
//	massmatrix(V, TF, mmtype, M);
//
//	invert_diag(M, Mi);
//
//	// Biharmonic operator
//	SparseMatrix<typename DerivedW::Scalar> Q = L.transpose() * Mi * L;
//
//
//
//	igl::min_quad_with_fixed_data<double> precomp_data;
//
//	return true;
//}

template<
	typename Derivedb,
	typename DerivedW >
	void reorder_matrix(
		const Eigen::PlainObjectBase<Derivedb>& b,
		Eigen::SparseMatrix<DerivedW>& W)
{
	using namespace std;
	using namespace Eigen;

	const int n = b.size();
	W.reserve(n);

	vector<Triplet<double> > IJV;

	for (size_t i = 0; i < n; i++)
		IJV.push_back(Triplet<double>(b(i), i, 1.));

	W.resize(n, n);
	W.setFromTriplets(IJV.begin(), IJV.end());
}






#ifdef IGL_STATIC_LIBRARY
// Explicit template specialization
template bool bilaplacian_coordinates<class Eigen::Matrix<double, -1, -1, 0, -1, -1>, class Eigen::Matrix<int, -1, -1, 0, -1, -1>, int, class Eigen::Matrix<double, -1, -1, 0, -1, -1> >(class Eigen::PlainObjectBase<class Eigen::Matrix<double, -1, -1, 0, -1, -1> > const &, class Eigen::PlainObjectBase<class Eigen::Matrix<int, -1, -1, 0, -1, -1> > const &, class std::vector<class std::vector<int, class std::allocator<int> >, class std::allocator<class std::vector<int, class std::allocator<int> > > > const &, class Eigen::PlainObjectBase<class Eigen::Matrix<double, -1, -1, 0, -1, -1> > &);
template bool bilaplacian_coordinates<class Eigen::Matrix<double, -1, -1, 0, -1, -1>, class Eigen::Matrix<int, -1, -1, 0, -1, -1>, class Eigen::Matrix<int, -1, 1, 0, -1, 1>, class Eigen::Matrix<double, -1, -1, 0, -1, -1> >(class Eigen::PlainObjectBase<class Eigen::Matrix<double, -1, -1, 0, -1, -1> > const &, class Eigen::PlainObjectBase<class Eigen::Matrix<int, -1, -1, 0, -1, -1> > const &, class Eigen::PlainObjectBase<class Eigen::Matrix<int, -1, 1, 0, -1, 1> > const &, class Eigen::PlainObjectBase<class Eigen::Matrix<double, -1, -1, 0, -1, -1> > &);

template void reorder_matrix<class Eigen::Matrix<int, -1, 1, 0, -1, 1>, double>(class Eigen::PlainObjectBase<class Eigen::Matrix<int, -1, 1, 0, -1, 1> > const &, class Eigen::SparseMatrix<double, 0, int> &);

#endif