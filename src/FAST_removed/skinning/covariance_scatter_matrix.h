#ifndef IGL_COVARIANCE_SCATTER_MATRIX_H
#define IGL_COVARIANCE_SCATTER_MATRIX_H

#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include "ArapEnergy.h"
 
// Construct the covariance scatter matrix for a given arap energy
// Inputs:
//   V  #V by dim list of initial domain positions
//   F  #F by 3 list of triangle indices into V
//   energy  ArapEnergy enum value defining which energy is being used. See
//     ArapEnergy.h for valid options and explanations.
// Outputs:
//   CSM dim*#V by dim*#V sparse matrix containing special laplacians along the
//     diagonal so that when multiplied by V gives covariance matrix elements,
//     can be used to speed up covariance matrix computation
inline void covariance_scatter_matrix(
  const Eigen::MatrixXd & V, 
  const Eigen::MatrixXi & F,
  const ArapEnergy energy,
  Eigen::SparseMatrix<double>& CSM);

// Implementation
#include <igl/cotmatrix.h>
#include <igl/diag.h>
#include <igl/sum.h>
#include <igl/edges.h>
#include <igl/verbose.h>
#include <igl/cat.h>
#include "arap_linear_block.h"

inline void covariance_scatter_matrix(
  const Eigen::MatrixXd & V, 
  const Eigen::MatrixXi & F,
  const ArapEnergy energy,
  Eigen::SparseMatrix<double>& CSM)
{
  using namespace igl;
  using namespace Eigen;
  // number of mesh vertices
  int n = V.rows();
  assert(n > F.maxCoeff());
  // dimension of mesh
  int dim = V.cols();
  // Number of mesh elements
  int m = F.rows();

  // number of rotations
  int nr;
  switch(energy)
  {
    case ARAP_SPOKES:
      nr = n;
      break;
    case ARAP_SPOKES_AND_RIMS:
      nr = n;
      break;
    case ARAP_ELEMENTS:
      nr = m;
      break;
    default:
      fprintf(
        stderr,
        "covariance_scatter_matrix.h: Error: Unsupported arap energy %d\n",
        energy);
      return;
  }

  SparseMatrix<double> KX,KY,KZ;
  arap_linear_block(V,F,0,energy,KX);
  arap_linear_block(V,F,1,energy,KY);
  SparseMatrix<double> Z(n,nr);
  if(dim == 2)
  {
    CSM = cat(1,cat(2,KX,Z),cat(2,Z,KY)).transpose();
  }else if(dim == 3)
  {
    arap_linear_block(V,F,2,energy,KZ);
    SparseMatrix<double>ZZ(n,nr*2);
    CSM = 
      cat(1,cat(1,cat(2,KX,ZZ),cat(2,cat(2,Z,KY),Z)),cat(2,ZZ,KZ)).transpose();
  }else
  {
    fprintf(
     stderr,
     "covariance_scatter_matrix.h: Error: Unsupported dimension %d\n",
     dim);
    return;
  }

}
#endif
