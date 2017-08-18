#ifndef ARAP_RHS_H
#define ARAP_RHS_H

#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include "ArapEnergy.h"

// ARAP_RHS build right-hand side constructor of global poisson solve for
// various Arap energies
// Inputs:
//   V  #V by dim list of initial domain positions
//   F  #F by 3 list of triangle indices into V
//   energy  ArapEnergy enum value defining which energy is being used. See
//     ArapEnergy.h for valid options and explanations.
// Outputs:
//   K  #V*dim by #(F|V)*dim*dim matrix such that: 
//     b = K * reshape(permute(R,[3 1 2]),size(V|F,1)*size(V,2)*size(V,2),1);
//   
inline void arap_rhs(
  const Eigen::MatrixXd & V, 
  const Eigen::MatrixXi & F,
  const ArapEnergy energy,
  Eigen::SparseMatrix<double>& K);

// Implementation
#include <igl/verbose.h>
#include <igl/cotmatrix.h>
#include <igl/repdiag.h>

inline void arap_rhs(
  const Eigen::MatrixXd & V, 
  const Eigen::MatrixXi & F,
  const ArapEnergy energy,
  Eigen::SparseMatrix<double>& K)
{
  using namespace igl;
  using namespace Eigen;
  // Number of dimensions
  int dim = V.cols();
  // Number of mesh vertices
  int n = V.rows();
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
  if(dim == 2)
  {
    K = cat(2,repdiag(KX,dim),repdiag(KY,dim));
  }else if(dim == 3)
  {
    arap_linear_block(V,F,2,energy,KZ);
    K = cat(2,cat(2,repdiag(KX,dim),repdiag(KY,dim)),repdiag(KZ,dim));
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
