#ifndef ARAP_DOF_H
#define ARAP_DOF_H

#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <igl/min_quad_with_fixed.h>
#include "ArapEnergy.h"
#include <vector>

// Caller example:
//
// Once:
// arap_dof_precomputation(...)
//
// Each frame:
// while(not satisfied)
//   arap_dof_update(...)
// end

struct arap_dof_data;

///////////////////////////////////////////////////////////////////////////
//
// Arap DOF precomputation consists of two parts the computation. The first is
// that which depends solely on the mesh (V,F), the linear blend skinning
// weights (M) and the groups G. Then there's the part that depends on the
// previous precomputation and the list of free and fixed vertices. 
//
///////////////////////////////////////////////////////////////////////////

// Precomputes the system we are going to optimize. This consists of building
// constructor matrices (to compute covariance matrices from transformations
// and to build the poisson solve right hand side from rotation matrix entries)
// and also prefactoring the poisson system.
//
// Inputs:
//   V  #V by dim list of vertex positions
//   F  #F by {3|4} list of face indices
//   M  #V * dim by #handles * dim * (dim+1) matrix such that
//     new_V(:) = LBS(V,W,A) = reshape(M * A,size(V)), where A is a column
//     vectors formed by the entries in each handle's dim by dim+1 
//     transformation matrix. Specifcally, A =
//       reshape(permute(Astack,[3 1 2]),n*dim*(dim+1),1)
//     or A = [Lxx;Lyx;Lxy;Lyy;tx;ty], and likewise for other dim
//     if Astack(:,:,i) is the dim by (dim+1) transformation at handle i
//     handles are ordered according to P then BE (point handles before bone
//     handles)
//   G  #V list of group indices (1 to k) for each vertex, such that vertex i 
//     is assigned to group G(i)
//   fixed  list of weight indices for fixed handles: handles whose
//     transformation is completely fixed, that is no degrees of freedom. This
//     is not necessarily the complement of 'free'
//		NOTE: the constraints for fixed transformations still need to be present in A_eq
//   Aeq  dim*#constraint_points by m*dim*(dim+1)  matrix of linear equality
//     constraint coefficients. Each row corresponds to a linear constraint, so
//     that Aeq * L = Beq says that the linear transformation entries in the
//     column L should produce the user supplied positional constraints for
//     each handle in Beq. The row Aeq(i*dim+d) corresponds to the constrain on
//     coordinate d of position i
// Outputs:
//   data  structure containing all necessary precomputation for calling
//     arap_dof_update
// Returns true on success, false on error
//
// See also: lbs_matrix
bool arap_dof_precomputation(
  const Eigen::MatrixXd & V, 
  const Eigen::MatrixXi & F,
  const Eigen::SparseMatrix<double> & M,
  const Eigen::Matrix<int,Eigen::Dynamic,1> & G,
  const Eigen::Matrix<int,Eigen::Dynamic,1> & fixed,
  const Eigen::SparseMatrix<double> & A_eq,
  arap_dof_data & data);

// Should always be called after arap_dof_precomputation, but may be called in
// between successive calls to arap_dof_update, recomputes precomputation given
// that there are only changes in free and fixed
//
// Inputs:
//   fixed  list of weight indices for fixed handles: handles whose
//     transformation is completely fixed, that is no degrees of freedom. This
//     is not necessarily the complement of 'free'
//		NOTE: the constraints for fixed transformations still need to be present in A_eq
//   A_eq  dim*#constraint_points by m*dim*(dim+1)  matrix of linear equality
//     constraint coefficients. Each row corresponds to a linear constraint, so
//     that A_eq * L = Beq says that the linear transformation entries in the
//     column L should produce the user supplied positional constraints for
//     each handle in Beq. The row A_eq(i*dim+d) corresponds to the constrain on
//     coordinate d of position i
// Outputs:
//   data  structure containing all necessary precomputation for calling
//     arap_dof_update
// Returns true on success, false on error
//
// See also: lbs_matrix
bool arap_dof_recomputation(
  const Eigen::Matrix<int,Eigen::Dynamic,1> & fixed,
  const Eigen::SparseMatrix<double> & A_eq,
  arap_dof_data & data);

// Optimizes the transformations attached to each weight function based on
// precomputed system.
//
// Inputs:
//   data  precomputation data struct output from arap_dof_precomputation
//   Beq  dim*#constraint_points constraint values.
//   L0  #handles * dim * dim+1 list of initial guess transformation entries,
//     also holds fixed transformation entries for fixed handles
//   max_iters  maximum number of iterations
//   tol  stopping critera parameter. If variables (linear transformation
//     matrix entries) change by less than 'tol' the optimization terminates,
//       0.75 (weak tolerance)
//       0.0 (extreme tolerance)
// Outputs:
//   L  #handles * dim * dim+1 list of final optimized transformation entries,
//     allowed to be the same as L
bool arap_dof_update(
  const arap_dof_data & data,
  const Eigen::Matrix<double,Eigen::Dynamic,1> & B_eq,
  const Eigen::MatrixXd & L0,
  const int max_iters,
  const double tol,
  Eigen::MatrixXd & L
  );

// Structure that contains fields for all precomputed data or data that needs
// to be remembered at update
struct arap_dof_data
{
  // Type of arap energy we're solving
  ArapEnergy energy;
  // LU decomposition precomptation data; note: not used by araf_dop_update any more, replaced by M_FullSolve
  igl::min_quad_with_fixed_data<double> lu_data;
  // List of indices of fixed transformation entries
  Eigen::Matrix<int,Eigen::Dynamic,1> fixed_dim;
  // List of precomputed covariance scatter matrices multiplied by lbs matrices
  //std::vector<Eigen::SparseMatrix<double> > CSM_M;
  std::vector<Eigen::MatrixXd> CSM_M;
  Eigen::SparseMatrix<double> M_KG;
  // Number of mesh vertices
  int n;
  // Number of weight functions
  int m;
  // Number of dimensions
  int dim;
  // Effective dimensions
  int effective_dim;
  // List of indices into C of positional constraints
  Eigen::Matrix<int,Eigen::Dynamic,1> interpolated;
  std::vector<bool> free_mask;
  // Full quadratic coefficients matrix before lagrangian
  Eigen::SparseMatrix<double> Q;

  //// Solve matrix for the global step
  //Eigen::MatrixXd M_Solve; // TODO: remove from here

  // Full solve matrix that contains also conversion from rotations to the right hand side, 
  // i.e., solves Poisson transformations just from rotations and positional constraints
  Eigen::MatrixXf M_FullSolve;

  // Precomputed condensed matrices (3x3 commutators folded to 1x1):
  Eigen::MatrixXf CSM;
  Eigen::MatrixXf CSolveBlock1;

  // Print timings at each update
  bool print_timings;
  // Default values
  arap_dof_data(): energy(ARAP_SPOKES)
  {
  }
};
#endif
