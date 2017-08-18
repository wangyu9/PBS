#ifndef UNIFORM_SAMPLE_H
#define UNIFORM_SAMPLE_H

#include <Eigen/Dense>
#include "Vec.h"
// UNIFORM_SAMPLE Sample a weight space mesh uniformly by relaxation
//
// Inputs:
//   W  #W by dim positions of mesh in weight space
//   F  #F by 3 indices of triangles
//   k  number of samplse
//   push  factor by which corners should be pushed away
// Outputs
//   WS  k by dim locations in weights space
//
void uniform_sample(
  const Eigen::MatrixXd & W,
  const Eigen::MatrixXi & F, 
  const int k, 
  const double push,
  Eigen::MatrixXd & WS);
#endif
