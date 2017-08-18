#ifndef POLAR_DEC
#define POLAR_DEC

// Computes the polar decomposition (R,T) of a matrix A
// Inputs:
//   A  3 by 3 matrix to be decomposed
// Outputs:
//   R  3 by 3 rotation matrix part of decomposition
//   T  3 by 3 stretch matrix part of decomposition
//
// Note: I'm not sure if this implementation is check against reflections in R
// Note: It is not
//
template<typename Mat>
inline void polar_dec(const Mat& A, Mat& R, Mat& T);

//Implementation
#include "polar_svd.h"
#include <Eigen/Core>
#ifdef _WIN32
#else
#  include <fenv.h>
#endif
// You will need the development version of Eigen which is > 3.0.3
// You can determine if you have computeDirect by issuing
//   grep -r computeDirect path/to/eigen/*
#define EIGEN_HAS_COMPUTE_DIRECT

// From Olga's CGAL mentee's ARAP code
template<typename Mat>
inline void polar_dec(const Mat& A, Mat& R, Mat& T)
{
#ifdef EIGEN_HAS_COMPUTE_DIRECT
 typedef typename Mat::Scalar Scalar;
 typedef Eigen::Matrix<typename Mat::Scalar,3,1> Vec;

 const Scalar th = std::sqrt(Eigen::NumTraits<Scalar>::dummy_precision());

 Eigen::SelfAdjointEigenSolver<Mat> eig;
 feclearexcept(FE_UNDERFLOW);
 eig.computeDirect(A.transpose()*A);
 if(fetestexcept(FE_UNDERFLOW) || eig.eigenvalues()(0)/eig.eigenvalues()(2)<th)
   return polar_svd(A,R,T);

 Vec S = eig.eigenvalues().cwiseSqrt();

 T = eig.eigenvectors() * S.asDiagonal() * eig.eigenvectors().transpose();
 R = A  * eig.eigenvectors() * S.asDiagonal().inverse()
        * eig.eigenvectors().transpose();

 if(std::abs(R.squaredNorm()-3.) > th)
   return polar_svd(A,R,T);
#else
  return polar_svd(A,R,T);
#endif
}
#endif
