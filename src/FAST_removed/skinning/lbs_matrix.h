#ifndef IGL_LBS_MATRIX_H
#define IGL_LBS_MATRIX_H

#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET
#include <Eigen/Dense>
#include <Eigen/Sparse>

// LBS_MATRIX  construct a matrix that when multiplied against a column of
// affine transformation entries computes new coordinates of the vertices
//
// I'm not sure it makes since that the result is stored as a sparse matrix.
// The number of non-zeros per row *is* dependent on the number of mesh
// vertices and handles.
//
// Inputs:
//   V  #V by dim list of vertex rest positions
//   W  #V by #handles list of correspondence weights
// Output:
//   M  #V * dim by #handles * dim * (dim+1) matrix such that
//     new_V(:) = LBS(V,W,A) = reshape(M * A,size(V)), where A is a column
//     vectors formed by the entries in each handle's dim by dim+1 
//     transformation matrix. Specifcally, A =
//       reshape(permute(Astack,[3 1 2]),n*dim*(dim+1),1)
//     or A = [Lxx;Lyx;Lxy;Lyy;tx;ty], and likewise for other dim
//     if Astack(:,:,i) is the dim by (dim+1) transformation at handle i
//
inline void lbs_matrix(
  const Eigen::MatrixXd & V, 
  const Eigen::MatrixXd & W,
  Eigen::SparseMatrix<double>& M);
// Same as LBS_MATRIX above but instead of giving W as a full matrix of weights
// (each vertex has #handles weights), a constant number of weights are given
// for each vertex.
// 
// Inputs:
//   V  #V by dim list of vertex rest positions
//   W  #V by k  list of k correspondence weights per vertex
//   WI  #V by k  list of k correspondence weight indices per vertex. Such that
//     W(j,WI(i)) gives the ith most significant correspondence weight on vertex j
// Output:
//   M  #V * dim by #handles * dim * (dim+1) matrix such that
//     new_V(:) = LBS(V,W,A) = reshape(M * A,size(V)), where A is a column
//     vectors formed by the entries in each handle's dim by dim+1 
//     transformation matrix. Specifcally, A =
//       reshape(permute(Astack,[3 1 2]),n*dim*(dim+1),1)
//     or A = [Lxx;Lyx;Lxy;Lyy;tx;ty], and likewise for other dim
//     if Astack(:,:,i) is the dim by (dim+1) transformation at handle i
//
inline void lbs_matrix(
  const Eigen::MatrixXd & V, 
  const Eigen::MatrixXd & W,
  const Eigen::MatrixXi & WI,
  Eigen::SparseMatrix<double>& M);

// Implementation
inline void lbs_matrix(
  const Eigen::MatrixXd & V, 
  const Eigen::MatrixXd & W,
  Eigen::SparseMatrix<double>& M)
{
  // number of mesh vertices
  int n = V.rows();
  assert(n == W.rows());
  // dimension of mesh
  int dim = V.cols();
  // number of handles
  int m = W.cols();

  Eigen::DynamicSparseMatrix<double, Eigen::RowMajor> 
    dyn_M(n*dim,m*dim*(dim+1));

  // loop over coordinates of mesh vertices
  for(int x = 0; x < dim; x++)
  {
    // loop over mesh vertices
    for(int j = 0; j < n; j++)
    {
      // loop over handles
      for(int i = 0; i < m; i++)
      {
        // loop over cols of affine transformations
        for(int c = 0; c < (dim+1); c++)
        {
          double value = W(j,i);
          if(c<dim)
          {
            value *= V(j,c);
          }
          dyn_M.coeffRef(x*n + j,x*m + c*m*dim + i) = value;
        }
      }
    }
  }

  M = Eigen::SparseMatrix<double>(dyn_M);
}

inline void lbs_matrix(
  const Eigen::MatrixXd & V, 
  const Eigen::MatrixXd & W,
  const Eigen::MatrixXi & WI,
  Eigen::SparseMatrix<double>& M)
{
  // number of mesh vertices
  int n = V.rows();
  assert(n == W.rows());
  assert(n == WI.rows());
  // dimension of mesh
  int dim = V.cols();
  // number of handles
  int m = WI.maxCoeff()+1;
  // max number of influencing handles
  int k = W.cols();
  assert(k == WI.cols());

  Eigen::DynamicSparseMatrix<double, Eigen::RowMajor> 
    dyn_M(n*dim,m*dim*(dim+1));

  // loop over coordinates of mesh vertices
  for(int x = 0; x < dim; x++)
  {
    // loop over mesh vertices
    for(int j = 0; j < n; j++)
    {
      // loop over handles
      for(int i = 0; i < k; i++)
      {
        // loop over cols of affine transformations
        for(int c = 0; c < (dim+1); c++)
        {
          double value = W(j,i);
          if(c<dim)
          {
            value *= V(j,c);
          }
          if(value != 0)
          {
            dyn_M.coeffRef(x*n + j,x*m + c*m*dim + WI(j,i)) = value;
          }
        }
      }
    }
  }

  M = Eigen::SparseMatrix<double>(dyn_M);
}

#endif
