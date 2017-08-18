#include "facet_laplacian.h"

//#include <igl/cotmatrix.h>

// For error printing
#include <cstdio>
//#include <igl/cotangent.h>
#include <cotangent.h>//This is removed from igl

#include <igl/sort.h>
#include <igl/unique.h>

#include <igl/doublearea.h>
//#include "volume.h"
#include <igl/volume.h>

//for debugging output
//#include <igl/writeDMAT.h>

// Bug in unsupported/Eigen/SparseExtra needs iostream first
#include <iostream>
#include <unsupported/Eigen/SparseExtra>


// outputs:
// EF: facets: edges in 2D, faces in 3D
template <typename DerivedV, typename DerivedTF, typename DerivedEF, typename Scalar, typename ScalarM>
IGL_INLINE void facet_laplacian(
	const Eigen::PlainObjectBase<DerivedV> & V, 
	const Eigen::PlainObjectBase<DerivedTF> & TF,
	Eigen::SparseMatrix<Scalar>& L,
	Eigen::SparseMatrix<ScalarM>& M,
	Eigen::PlainObjectBase<DerivedEF> & EF
	)
{
	using namespace igl;
	using namespace Eigen;
	Eigen::DynamicSparseMatrix<double> foo;

	int n = V.rows();
	int tf = TF.rows();

	int dim = TF.cols();
	// 3 for triangles, 4 for tets
	assert(dim == 3 || dim == 4);

	Matrix<int,Dynamic,Dynamic> facets;
	VectorXi cot_order;
	if(dim == 3)
	{//edge is dim by dim-1
		facets.resize(3,2);
		facets << 
			1,2,
			2,0,
			0,1;
		cot_order.resize(3);
		cot_order <<
			0,
			1,
			2;
		//corresponds to order [1,2],[2,0],[0,1]
	}else if(dim == 4)
	{
		facets.resize(4,3);
		facets <<
			1,3,2,
			0,2,3,
			0,3,1,
			0,1,2;
		cot_order.resize(4,4);
		cot_order <<
			-1, 2, 1, 3,
			 2,-1, 0, 4,
			 1, 0,-1, 5,
			 3, 4, 5,-1;
		//corresponds to order:
	}else
	{
		return;
	}

	Eigen::PlainObjectBase<DerivedTF> facet_map;
	facet_map.resize(dim*tf,1);
	Eigen::PlainObjectBase<DerivedTF> duplicated_facets;
	duplicated_facets.resize(dim*tf,dim-1);
	for (int i=0; i<facets.rows(); i++)
	{
		for (int j=0; j<facets.cols(); j++)
		{
			int e = facets(i,j);
			duplicated_facets.block(i*tf,j,tf,1) = TF.block(0,e,tf,1);
		}
	}

	Eigen::PlainObjectBase<DerivedTF> sorted_duplicated_facets;
	sorted_duplicated_facets.resize(dim*tf,dim-1);

	Eigen::PlainObjectBase<DerivedTF> IX;
	sort_new(duplicated_facets, 2, true, sorted_duplicated_facets, IX);

	Eigen::PlainObjectBase<DerivedTF> IA;
	Eigen::PlainObjectBase<DerivedTF> IC;

	unique_rows(sorted_duplicated_facets, EF, IA, IC);
	int num_facets = EF.rows();

	// test output
	//igl::writeDMAT("TF.dmat",TF);
	//igl::writeDMAT("duplicated_facets.dmat",duplicated_facets);
	//igl::writeDMAT("EF.dmat",EF);

	DynamicSparseMatrix<Scalar, RowMajor> dyn_L (dim*tf, V.rows());
	//DynamicSparseMatrix<Scalar, RowMajor> dyn_L (num_facets, V.rows());

	if (dim==3)
	{
		// This is important! it could decrease the comptuation time by a factor of 2
		// Laplacian for a closed 2d manifold mesh will have on average 7 entries per
		// row
		dyn_L.reserve(2*10*num_facets);
	} 
	else if(dim==4)
	{
		dyn_L.reserve(2*17*num_facets);
	}
	else
	{
		return;
	}


	// Gather cotangents
	Matrix<Scalar,Dynamic,Dynamic> C;
	cotangent(V,TF,C);

	igl::writeDMAT("C.dmat",C);

	// Loop over triangles
	for(int i = 0; i < TF.rows(); i++)
	{
		// loop over all facets int the element
		for(int j = 0; j<facets.rows(); j++)
		{
			int index = TF.rows()*j + i;//before unique op indexing
			//int fIndex = IA(index,0);// after unique op indexing, index in uniqued facets
			if (dim==3)
			{
				int a = facets(j,0);
				int b = facets(j,1);
				int c = 0+1+2-a-b;
				dyn_L.coeffRef(index,TF(i,a)) += C(i,cot_order(a));
				dyn_L.coeffRef(index,TF(i,b)) += C(i,cot_order(b));
				dyn_L.coeffRef(index,TF(i,c)) -= (C(i,cot_order(a))
												+ C(i,cot_order(b)));
			}
			else
			{
				int a = facets(j,0);
				int b = facets(j,1);
				int c = facets(j,2);
				int d = 0+1+2+3-a-b-c;
				dyn_L.coeffRef(index,TF(i,a)) -= C(i,cot_order(a,d));
				dyn_L.coeffRef(index,TF(i,b)) -= C(i,cot_order(b,d));
				dyn_L.coeffRef(index,TF(i,c)) -= C(i,cot_order(c,d));
				dyn_L.coeffRef(index,TF(i,d)) += C(i,cot_order(a,d))
												+ C(i,cot_order(b,d)) + C(i,cot_order(c,d));
			}
		}
	}
	// Corner indices of this triangle
	L = SparseMatrix<Scalar>(dyn_L);

	SparseMatrix<Scalar> MapMatrix(num_facets,dim*tf);
	std::vector<Triplet<Scalar> > triplets;
	for (long i=0; i<dim*tf; i++)
	{
		triplets.push_back(Triplet<Scalar>(IC(i,0),i,1));
	}
	MapMatrix.setFromTriplets(triplets.begin(), triplets.end());

	L = MapMatrix * L;


	//crouzeix_raviart_massmatrix
	M.resize(num_facets,num_facets);
	std::vector<Triplet<ScalarM> > M_triplets;
	MatrixXd masses;
	if (dim==3)
	{
		//MatrixXd dbla;
		VectorXd dbla;
		doublearea(V,TF,dbla);
		dbla = dbla/2.0;
		masses = dbla/3.0;
	} 
	else if (dim==4)
	{
		Eigen::MatrixXd V2 = V.cast<double>();
		Eigen::MatrixXd TF2 = TF.cast<double>();
		VectorXd vol;
		volume(V2,TF2,vol);
		masses = vol/4.0;
	}
	else
	{
		return;
	}

	// Loop over triangles
	for(int i = 0; i < TF.rows(); i++)
	{
		// loop over all facets int the element
		for(int j = 0; j<facets.rows(); j++)
		{
			int index = TF.rows()*j + i;//before unique op indexing
			triplets.push_back(Triplet<Scalar>(IC(index,0),IC(index,0),masses(i)));
		}
	}
}



// This version is not correct with the IA indexing

//// outputs:
//// EF: facets: edges in 2D, faces in 3D
//template <typename DerivedV, typename DerivedTF, typename DerivedEF, typename Scalar>
//IGL_INLINE void facet_laplacian(
//	const Eigen::PlainObjectBase<DerivedV> & V, 
//	const Eigen::PlainObjectBase<DerivedTF> & TF,
//	Eigen::SparseMatrix<Scalar>& L,
//	Eigen::PlainObjectBase<DerivedEF> & EF
//	)
//{
//	using namespace igl;
//	using namespace Eigen;
//	Eigen::DynamicSparseMatrix<double> foo;
//
//	int n = V.rows();
//	int tf = TF.rows();
//
//	int dim = TF.cols();
//	// 3 for triangles, 4 for tets
//	assert(dim == 3 || dim == 4);
//
//	Matrix<int,Dynamic,Dynamic> facets;
//	VectorXi cot_order;
//	if(dim == 3)
//	{//edge is dim by dim-1
//		facets.resize(3,2);
//		facets << 
//			1,2,
//			2,0,
//			0,1;
//		cot_order.resize(3);
//		cot_order <<
//			0,
//			1,
//			2;
//		//corresponds to order [1,2],[2,0],[0,1]
//	}else if(dim == 4)
//	{
//		facets.resize(4,3);
//		facets <<
//			1,3,2,
//			0,2,3,
//			0,3,1,
//			0,1,2;
//		cot_order.resize(4);
//		cot_order <<
//			0,
//			1,
//			2,
//			3;
//		//corresponds to order:
//	}else
//	{
//		return;
//	}
//
//	Eigen::PlainObjectBase<DerivedTF> facet_map;
//	facet_map.resize(dim*tf,1);
//	Eigen::PlainObjectBase<DerivedTF> duplicated_facets;
//	duplicated_facets.resize(dim*tf,dim-1);
//	for (int i=0; i<facets.rows(); i++)
//	{
//		for (int j=0; j<facets.cols(); j++)
//		{
//			int e = facets(i,j);
//			duplicated_facets.block(i*tf,j,tf,1) = TF.block(0,e,tf,1);
//		}
//	}
//
//	Eigen::PlainObjectBase<DerivedTF> sorted_duplicated_facets;
//	sorted_duplicated_facets.resize(dim*tf,dim-1);
//
//	Eigen::PlainObjectBase<DerivedTF> IX;
//	sort_new(duplicated_facets, 2, true, sorted_duplicated_facets, IX);
//
//	Eigen::PlainObjectBase<DerivedTF> IA;
//	Eigen::PlainObjectBase<DerivedTF> IC;
//
//	unique_plus(sorted_duplicated_facets, EF, IA, IC);
//	int num_facets = EF.rows();
//
//	// test output
//	//igl::writeDMAT("TF.dmat",TF);
//	//igl::writeDMAT("duplicated_facets.dmat",duplicated_facets);
//	//igl::writeDMAT("EF.dmat",EF);
//
//	DynamicSparseMatrix<Scalar, RowMajor> dyn_L (num_facets, V.rows());
//
//	if (dim==3)
//	{
//		// This is important! it could decrease the comptuation time by a factor of 2
//		// Laplacian for a closed 2d manifold mesh will have on average 7 entries per
//		// row
//		dyn_L.reserve(10*num_facets);
//	} 
//	else if(dim==4)
//	{
//		dyn_L.reserve(17*num_facets);
//	}
//	else
//	{
//		return;
//	}
//	
//
//	// Gather cotangents
//	Matrix<Scalar,Dynamic,Dynamic> C;
//	cotangent(V,TF,C);
//
//	// Loop over triangles
//	for(int i = 0; i < TF.rows(); i++)
//	{
//		// loop over all facets int the element
//		for(int j = 0; j<facets.rows(); j++)
//		{
//			int index = TF.rows()*j + i;//before unique op indexing
//			int fIndex = IA(index,0);// after unique op indexing, index in uniqued facets
//			if (dim==3)
//			{
//				int a = facets(j,0);
//				int b = facets(j,1);
//				int c = 0+1+2-a-b;
//				dyn_L.coeffRef(fIndex,EF(fIndex,0)) += C(i,cot_order(a));
//				dyn_L.coeffRef(fIndex,EF(fIndex,1)) += C(i,cot_order(b));
//				dyn_L.coeffRef(fIndex,EF(fIndex,2)) -= C(i,cot_order(a))
//													 + C(i,cot_order(b));
//			}
//			else
//			{
//				int a = facets(j,0);
//				int b = facets(j,1);
//				int c = facets(j,2);
//				int d = 0+1+2+3-a-b-c;
//				dyn_L.coeffRef(fIndex,TF(i,0)) += C(i,cot_order(a));
//				dyn_L.coeffRef(fIndex,TF(i,1)) += C(i,cot_order(b));
//				dyn_L.coeffRef(fIndex,TF(i,2)) += C(i,cot_order(c));
//				dyn_L.coeffRef(fIndex,TF(i,3)) -= C(i,cot_order(a))
//					+ C(i,cot_order(b)) + C(i,cot_order(c));
//			}
//		}
//	}
//	// Corner indices of this triangle
//	L = SparseMatrix<Scalar>(dyn_L);
//}


#ifndef IGL_HEADER_ONLY
// Explicit template specialization
// modified from: template void igl::cotmatrix<Eigen::Matrix<double, -1, -1, 0, -1, -1>, Eigen::Matrix<int, -1, -1, 0, -1, -1>, double>(Eigen::PlainObjectBase<Eigen::Matrix<double, -1, -1, 0, -1, -1> > const&, Eigen::PlainObjectBase<Eigen::Matrix<int, -1, -1, 0, -1, -1> > const&, Eigen::SparseMatrix<double, 0, int>&);

template void facet_laplacian
	<
	Eigen::Matrix<double, -1, -1, 0, -1, -1>, 
	Eigen::Matrix<int, -1, -1, 0, -1, -1>, 
	double,
	double,
	Eigen::Matrix<int, -1, -1, 0, -1, -1>
	>
		(
		Eigen::PlainObjectBase<Eigen::Matrix<double, -1, -1, 0, -1, -1> > const&, 
		Eigen::PlainObjectBase<Eigen::Matrix<int, -1, -1, 0, -1, -1> > const&, 
		Eigen::SparseMatrix<double, 0, int>&,
		Eigen::SparseMatrix<double, 0, int>&,
		Eigen::PlainObjectBase<Eigen::Matrix<int, -1, -1, 0, -1, -1> > const&
		);

#endif