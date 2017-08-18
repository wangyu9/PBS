#include <Eigen/Dense>
// "Columnize" a stack of block matrices. If A = [A1,A2,A3,...,Ak] with each A*
// an m by n block then this produces the column vector whose entries are 
// B(j*m*k+i*k+b) = A(i,b*n+j);
// or if A = [A1;A2;...;Ak] then
// B(j*m*k+i*k+b) = A(i+b*m,j);
//
// Templates:
//   T  should be a eigen matrix primitive type like int or double
// Inputs:
//   A  m*k by n (dim: 1) or m by n*k (dim: 2) eigen Matrix of type T values
//   k  number of blocks
//   dim  dimension in which blocks are stacked
// Output
//   B  m*n*k eigen vector of type T values,
//
// See also: transpose_blocks
template <typename T,const int C>
inline void columnize(
  const Eigen::Matrix<T,Eigen::Dynamic,Eigen::Dynamic> & A,
  const size_t k,
  const size_t dim,
  Eigen::Matrix<T,Eigen::Dynamic,C> & B);

// Implementation
#include <cassert>

template <typename T, const int C>
inline void columnize(
  const Eigen::Matrix<T,Eigen::Dynamic,Eigen::Dynamic> & A,
  const size_t k,
  const size_t dim,
  Eigen::Matrix<T,Eigen::Dynamic,C> & B)
{
  // Eigen matrices must be 2d so dim must be only 1 or 2
  assert(dim == 1 || dim == 2);

  // block height, width, and number of blocks
  int m,n;
  if(dim == 1)
  {
    m = A.rows()/k;
    assert(m*(int)k == (int)A.rows());
    n = A.cols();
  }else// dim == 2
  {
    m = A.rows();
    n = A.cols()/k;
    assert(n*(int)k == (int)A.cols());
  }

  // resize output
  B.resize(A.rows()*A.cols(),1);

  for(int b = 0;b<(int)k;b++)
  {
    for(int i = 0;i<m;i++)
    {
      for(int j = 0;j<n;j++)
      {
        if(dim == 1)
        {
          B(j*m*k+i*k+b) = A(i+b*m,j);
        }else
        {
          B(j*m*k+i*k+b) = A(i,b*n+j);
        }
      }
    }
  }
}
