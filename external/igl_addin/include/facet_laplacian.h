#ifndef IGL_ADDIN_FACET_LAPLACIAN_H
#define IGL_ADDIN_FACET_LAPLACIAN_H

#include <igl/igl_inline.h>
#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET
#include <Eigen/Dense>
#include <Eigen/Sparse>

// Constructs the facet laplacian for a given
// mesh (V,F).
//
// Templates:
//   DerivedV  derived type of eigen matrix for V (e.g. derived from
//     MatrixXd)
//   DerivedF  derived type of eigen matrix for F (e.g. derived from
//     MatrixXi)
//   Scalar  scalar type for eigen sparse matrix (e.g. double)
// Inputs:
//   V  #V by dim list of mesh vertex positions
//   F  #F by simplex_size list of mesh faces (must be triangles)
// Outputs: 
//   L  #V by #V cotangent matrix, each row i corresponding to V(i,:)
//

template <typename DerivedV, typename DerivedTF, typename DerivedEF, typename Scalar, typename ScalarM>
IGL_INLINE void facet_laplacian(
	const Eigen::PlainObjectBase<DerivedV> & V, 
	const Eigen::PlainObjectBase<DerivedTF> & TF, 
	Eigen::SparseMatrix<Scalar>& L,
	Eigen::SparseMatrix<ScalarM>& M,
	Eigen::PlainObjectBase<DerivedEF> & EF
	);

#ifdef IGL_HEADER_ONLY
#include "facet_laplacian.cpp"
#endif

#endif /*IGL_ADDIN_FACET_LAPLACIAN_H*/