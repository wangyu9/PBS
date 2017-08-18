#ifndef GROUP_SUM_MATRIX_H
#define GROUP_SUM_MATRIX_H

#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET
#include <Eigen/Dense>
#include <Eigen/Sparse>

// GROUP_SUM_MATRIX Builds a matrix A such that A*V computes the sum of
// vertices in each group specified by G
//
// group_sum_matrix(G,k,A);
// 
// Templates:
//   T  should be a eigen sparse matrix primitive type like int or double
// Inputs:
//   G  #V list of group indices (0 to k-1) for each vertex, such that vertex i 
//     is assigned to group G(i)
//   k  #groups, good choice is max(G)+1
// Outputs:
//   A  #groups by #V sparse matrix such that A*V = group_sums
//
template <typename T>
inline void group_sum_matrix(
  const Eigen::Matrix<int,Eigen::Dynamic,1> & G,
  const int k,
  Eigen::SparseMatrix<T>& A);

template <typename T>
inline void group_sum_matrix(
  const Eigen::Matrix<int,Eigen::Dynamic,1> & G,
  const int k,
  Eigen::SparseMatrix<T>& A)
{
  // number of vertices
  int n = G.rows();
  assert(k > G.maxCoeff());

  Eigen::DynamicSparseMatrix<T, Eigen::RowMajor> dyn_A(k,n);

  // builds A such that A(i,j) = 1 where i corresponds to group i and j
  // corresponds to vertex j

  // Loop over vertices
  for(int j = 0;j<n;j++)
  {
    dyn_A.coeffRef(G(j),j) = 1;
  }

  A = Eigen::SparseMatrix<T>(dyn_A);
}

#endif
