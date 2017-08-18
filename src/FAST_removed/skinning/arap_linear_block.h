#ifndef ARAP_LINEAR_BLOCK_H
#define ARAP_LINEAR_BLOCK_H

#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET
#include <Eigen/Sparse>
#include "ArapEnergy.h"

// ARAP_LINEAR_BLOCK constructs a block of the matrix which constructs the
// linear terms of a given arap energy. When treating rotations as knowns
// (arranged in a column) then this constructs Kd of K such that the linear
// portion of the energy is as a column:
//   K * R = [Kx Z  ... Ky Z  ... 
//            Z  Kx ... Z  Ky ... 
//            ... ]
// These blocks are also used to build the "covariance scatter matrices". Here
// we want to build a scatter matrix that multiplies against positions
// (treated as known) producing covariance matrices to fit each rotation.
// Notice that in the case of the RHS of the poisson solve the rotations are
// known and the positions unknown, and vice versa for rotation fitting. These
// linear block just relate the rotations to the positions, linearly in each.
//
// Templates:
//   MatV  vertex position matrix, e.g. Eigen::MatrixXd
//   MatF  face index matrix, e.g. Eigen::MatrixXd
//   Scalar  e.g. double
// Inputs:
//   V  #V by dim list of initial domain positions
//   F  #F by #simplex size list of triangle indices into V
//   d  coordinate of linear constructor to build
//   energy  ArapEnergy enum value defining which energy is being used. See
//     ArapEnergy.h for valid options and explanations.
// Outputs:
//   Kd  #V by #V/#F block of the linear constructor matrix corresponding to
//     coordinate d
//
template <typename MatV, typename MatF, typename Scalar>
void arap_linear_block(
  const MatV & V,
  const MatF & F,
  const int d,
  const ArapEnergy energy,
  Eigen::SparseMatrix<Scalar> & Kd);
// Helper functions for each energy type
template <typename MatV, typename MatF, typename Scalar>
void arap_linear_block_spokes(
  const MatV & V,
  const MatF & F,
  const int d,
  Eigen::SparseMatrix<Scalar> & Kd);
template <typename MatV, typename MatF, typename Scalar>
void arap_linear_block_spokes_and_rims(
  const MatV & V,
  const MatF & F,
  const int d,
  Eigen::SparseMatrix<Scalar> & Kd);
template <typename MatV, typename MatF, typename Scalar>
void arap_linear_block_elements(
  const MatV & V,
  const MatF & F,
  const int d,
  Eigen::SparseMatrix<Scalar> & Kd);

#endif
