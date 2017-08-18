#ifndef IGL_HEADER_ONLY
#define IGL_HEADER_ONLY
#endif

#include "arap_linear_block.h"
#include <igl/verbose.h>
#include <igl/cotangent.h>
#include <Eigen/Dense>
#include "C:\WorkSpace\Visual Studio 2010\lim\external\eigen-eigen-36bf2ceaf8f5\unsupported\Eigen\src\SparseExtra\DynamicSparseMatrix.h"
 
template <typename MatV, typename MatF, typename Scalar>
void arap_linear_block(
  const MatV & V,
  const MatF & F,
  const int d,
  const ArapEnergy energy,
  Eigen::SparseMatrix<Scalar> & Kd)
{
  using namespace igl;
  switch(energy)
  {
    case ARAP_SPOKES:
      return arap_linear_block_spokes(V,F,d,Kd);
      break;
    case ARAP_SPOKES_AND_RIMS:
      return arap_linear_block_spokes_and_rims(V,F,d,Kd);
      break;
    case ARAP_ELEMENTS:
      return arap_linear_block_elements(V,F,d,Kd);
      break;
    default:
      verbose("Unsupported energy type: %d\n",energy);
      assert(false);
  }
}


template <typename MatV, typename MatF, typename Scalar>
void arap_linear_block_spokes(
  const MatV & V,
  const MatF & F,
  const int d,
  Eigen::SparseMatrix<Scalar> & Kd)
{
  using namespace igl;
  using namespace std;
  using namespace Eigen;
  // simplex size (3: triangles, 4: tetrahedra)
  int simplex_size = F.cols();
  // Number of elements
  int m = F.rows();
  // Temporary output
  Eigen::DynamicSparseMatrix<Scalar, Eigen::RowMajor> 
    dyn_Kd(V.rows(), V.rows());
  Matrix<int,Dynamic,2> edges;
  if(simplex_size == 3)
  {
    // triangles
    dyn_Kd.reserve(7*V.rows());
    edges.resize(3,2);
    edges << 
      1,2,
      2,0,
      0,1;
  }else if(simplex_size == 4)
  {
    // tets
    dyn_Kd.reserve(17*V.rows());
    edges.resize(6,2);
    edges << 
      1,2,
      2,0,
      0,1,
      3,0,
      3,1,
      3,2;
  }
  // gather cotangent weights
  Matrix<Scalar,Dynamic,Dynamic> C;
  cotangent(V,F,C);
  // should have weights for each edge
  assert(C.cols() == edges.rows());
  // loop over elements
  for(int i = 0;i<m;i++)
  {
    // loop over edges of element
    for(int e = 0;e<edges.rows();e++)
    {
      int source = F(i,edges(e,0));
      int dest = F(i,edges(e,1));
      double v = 0.5*C(i,e)*(V(source,d)-V(dest,d));
      dyn_Kd.coeffRef(source,dest) += v;
      dyn_Kd.coeffRef(dest,source) += -v;
      dyn_Kd.coeffRef(source,source) += v;
      dyn_Kd.coeffRef(dest,dest) += -v;
    }
  }
  Kd = Eigen::SparseMatrix<double>(dyn_Kd);
}

template <typename MatV, typename MatF, typename Scalar>
void arap_linear_block_spokes_and_rims(
  const MatV & V,
  const MatF & F,
  const int d,
  Eigen::SparseMatrix<Scalar> & Kd)
{
  using namespace igl;
  using namespace std;
  using namespace Eigen;
  // simplex size (3: triangles, 4: tetrahedra)
  int simplex_size = F.cols();
  // Number of elements
  int m = F.rows();
  // Temporary output
  DynamicSparseMatrix<Scalar, Eigen::RowMajor> 
    dyn_Kd(V.rows(), V.rows());
  Matrix<int,Dynamic,2> edges;
  if(simplex_size == 3)
  {
    // triangles
    dyn_Kd.reserve(7*V.rows());
    edges.resize(3,2);
    edges << 
      1,2,
      2,0,
      0,1;
  }else if(simplex_size == 4)
  {
    // tets
    dyn_Kd.reserve(17*V.rows());
    edges.resize(6,2);
    edges << 
      1,2,
      2,0,
      0,1,
      3,0,
      3,1,
      3,2;
    // Not implemented yet for tets
    assert(false);
  }
  // gather cotangent weights
  Matrix<Scalar,Dynamic,Dynamic> C;
  cotangent(V,F,C);
  // should have weights for each edge
  assert(C.cols() == edges.rows());
  // loop over elements
  for(int i = 0;i<m;i++)
  {
    // loop over edges of element
    for(int e = 0;e<edges.rows();e++)
    {
      int source = F(i,edges(e,0));
      int dest = F(i,edges(e,1));
      double v = C(i,e)*(V(source,d)-V(dest,d))/3.0;
      // loop over edges again
      for(int f = 0;f<edges.rows();f++)
      {
        int Rs = F(i,edges(f,0));
        int Rd = F(i,edges(f,1));
        if(Rs == source && Rd == dest)
        {
          dyn_Kd.coeffRef(Rs,Rd) += v;
          dyn_Kd.coeffRef(Rd,Rs) += -v;
        }else if(Rd == source)
        {
          dyn_Kd.coeffRef(Rd,Rs) += v;
        }else if(Rs == dest)
        {
          dyn_Kd.coeffRef(Rs,Rd) += -v;
        }
      }
      dyn_Kd.coeffRef(source,source) += v;
      dyn_Kd.coeffRef(dest,dest) += -v;
    }
  }
  Kd = Eigen::SparseMatrix<double>(dyn_Kd);
}

template <typename MatV, typename MatF, typename Scalar>
void arap_linear_block_elements(
  const MatV & V,
  const MatF & F,
  const int d,
  Eigen::SparseMatrix<Scalar> & Kd)
{
  using namespace igl;
  using namespace std;
  using namespace Eigen;
  // simplex size (3: triangles, 4: tetrahedra)
  int simplex_size = F.cols();
  // Number of elements
  int m = F.rows();
  // Temporary output
  DynamicSparseMatrix<Scalar, Eigen::RowMajor> 
    dyn_Kd(V.rows(), F.rows());
  Matrix<int,Dynamic,2> edges;
  if(simplex_size == 3)
  {
    // triangles
    dyn_Kd.reserve(7*V.rows());
    edges.resize(3,2);
    edges << 
      1,2,
      2,0,
      0,1;
  }else if(simplex_size == 4)
  {
    // tets
    dyn_Kd.reserve(17*V.rows());
    edges.resize(6,2);
    edges << 
      1,2,
      2,0,
      0,1,
      3,0,
      3,1,
      3,2;
  }
  // gather cotangent weights
  Matrix<Scalar,Dynamic,Dynamic> C;
  cotangent(V,F,C);
  // should have weights for each edge
  assert(C.cols() == edges.rows());
  // loop over elements
  for(int i = 0;i<m;i++)
  {
    // loop over edges of element
    for(int e = 0;e<edges.rows();e++)
    {
      int source = F(i,edges(e,0));
      int dest = F(i,edges(e,1));
      double v = C(i,e)*(V(source,d)-V(dest,d));
      dyn_Kd.coeffRef(source,i) += v;
      dyn_Kd.coeffRef(dest,i) += -v;
    }
  }
  Kd = Eigen::SparseMatrix<double>(dyn_Kd);
}


// Explicit template specialization
template void arap_linear_block<Eigen::Matrix<double, -1, -1, 0, -1, -1>, Eigen::Matrix<int, -1, -1, 0, -1, -1>, double>(Eigen::Matrix<double, -1, -1, 0, -1, -1> const&, Eigen::Matrix<int, -1, -1, 0, -1, -1> const&, int, ArapEnergy, Eigen::SparseMatrix<double, 0, int>&);
