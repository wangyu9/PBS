#include "crouzeix_raviart_massmatrix.h"
//#include "normalize_row_sums.h"
//#include "sparse.h"
//#include "repmat.h"
#include <Eigen/Geometry>
#include <iostream>

template <typename DerivedV, typename DerivedF, typename Scalar>
IGL_INLINE void crouzeix_raviart_massmatrix(
  const Eigen::MatrixBase<DerivedV> & V, 
  const Eigen::MatrixBase<DerivedF> & F, 
  Eigen::SparseMatrix<Scalar>& M)
{
  using namespace Eigen;
  using namespace std;

  const int n = V.rows();
  const int m = F.rows();
  const int simplex_size = F.cols();
}

#ifndef IGL_HEADER_ONLY
// Explicit template specialization
template void igl::crouzeix_raviart_massmatrix<Eigen::Matrix<double, -1, -1, 0, -1, -1>, Eigen::Matrix<int, -1, -1, 0, -1, -1>, double>(Eigen::MatrixBase<Eigen::Matrix<double, -1, -1, 0, -1, -1> > const&, Eigen::MatrixBase<Eigen::Matrix<int, -1, -1, 0, -1, -1> > const&, igl::MassMatrixType, Eigen::SparseMatrix<double, 0, int>&);
#endif
