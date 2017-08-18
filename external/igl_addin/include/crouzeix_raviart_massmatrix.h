#ifndef IGL_ADDIN_CROUZEIX_RAVIART_MASSMATRIX_H
#define IGL_ADDIN_CROUZEIX_RAVIART_MASSMATRIX_H
#include <igl/igl_inline.h>

#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET
#include <Eigen/Dense>
#include <Eigen/Sparse>


// Constructs the mass (area) matrix for a given mesh (V,F).
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
//   M  #V by #V mass matrix
//
// See also: adjacency_matrix
//
template <typename DerivedV, typename DerivedF, typename Scalar>
IGL_INLINE void crouzeix_raviart_massmatrix(
const Eigen::MatrixBase<DerivedV> & V, 
const Eigen::MatrixBase<DerivedF> & F, 
Eigen::SparseMatrix<Scalar>& M);


#ifdef IGL_HEADER_ONLY
#  include "crouzeix_raviart_massmatrix.cpp"
#endif

#endif /*IGL_ADDIN_CROUZEIX_RAVIART_MASSMATRIX_H*/

