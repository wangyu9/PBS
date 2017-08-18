#include "crouzeix_raviart_cotmatrix.h"

#include <igl/cotmatrix.h>

// For error printing
#include <cstdio>
#include <igl/cotangent.h>

// Bug in unsupported/Eigen/SparseExtra needs iostream first
#include <iostream>
#include <unsupported/Eigen/SparseExtra>

template <typename DerivedV, typename DerivedF, typename Scalar>
IGL_INLINE void crouzeix_raviart_cotmatrix(
	const Eigen::PlainObjectBase<DerivedV> & V, 
	const Eigen::PlainObjectBase<DerivedF> & F, 
	Eigen::SparseMatrix<Scalar>& L)
{
	printf("Error: To be implemented!\n");

	using namespace igl;
	using namespace Eigen;
	Eigen::DynamicSparseMatrix<double> foo;

	DynamicSparseMatrix<Scalar, RowMajor> dyn_L (V.rows(), V.rows());
	Matrix<int,Dynamic,2> edges;
	int simplex_size = F.cols();
	// 3 for triangles, 4 for tets
	assert(simplex_size == 3 || simplex_size == 4);
	if(simplex_size == 3)
	{
		// This is important! it could decrease the comptuation time by a factor of 2
		// Laplacian for a closed 2d manifold mesh will have on average 7 entries per
		// row
		dyn_L.reserve(7*V.rows());
		edges.resize(3,2);
		edges << 
			1,2,
			2,0,
			0,1;
	}else if(simplex_size == 4)
	{
		dyn_L.reserve(17*V.rows());
		edges.resize(6,2);
		edges << 
			1,2,
			2,0,
			0,1,
			3,0,
			3,1,
			3,2;
	}else
	{
		return;
	}
	// Gather cotangents
	Matrix<Scalar,Dynamic,Dynamic> C;
	cotangent(V,F,C);

	// Loop over triangles
	for(int i = 0; i < F.rows(); i++)
	{
		// loop over edges of element
		for(int e = 0;e<edges.rows();e++)
		{
			int source = F(i,edges(e,0));
			int dest = F(i,edges(e,1));
			dyn_L.coeffRef(source,dest) += C(i,e);
			dyn_L.coeffRef(dest,source) += C(i,e);
			dyn_L.coeffRef(source,source) += -C(i,e);
			dyn_L.coeffRef(dest,dest) += -C(i,e);
		}
	}
	// Corner indices of this triangle
	L = SparseMatrix<Scalar>(dyn_L);
}