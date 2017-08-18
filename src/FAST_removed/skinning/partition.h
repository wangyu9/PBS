#ifndef PARTITION_H
#define PARTITION_H

#include <Eigen/Dense>

// PARTITION partition vertices into groups based on each
// vertex's vector: vertices with similar coordinates (close in 
// space) will be put in the same group.
//
// Inputs:
//   W  #W by dim coordinate matrix
//   k  desired number of groups default is dim
// Output:
//   G  #W list of group indices (1 to k) for each vertex, such that vertex i 
//     is assigned to group G(i)
//   S  k  list of seed vertices
//   D  #W list of squared distances for each vertex to it's corresponding
//     closest seed
inline void partition(
  const Eigen::MatrixXd & W,
  const int k,
  Eigen::Matrix<int,Eigen::Dynamic,1> & G,
  Eigen::Matrix<int,Eigen::Dynamic,1> & S,
  Eigen::Matrix<double,Eigen::Dynamic,1> & D);

// Implementation
#include <iostream>
#include <igl/mat_min.h>

inline void partition(
  const Eigen::MatrixXd & W,
  const int k,
  Eigen::Matrix<int,Eigen::Dynamic,1> & G,
  Eigen::Matrix<int,Eigen::Dynamic,1> & S,
  Eigen::Matrix<double,Eigen::Dynamic,1> & D)
{
  // number of mesh vertices
  int n = W.rows();

  // Resize output
  G.resize(n);
  S.resize(k);

  // "Randomly" choose first seed
  // Pick a vertex farthest from 0
  int s;
  (W.array().square().matrix()).rowwise().sum().maxCoeff(&s);

  S(0) = s;
  // Initialize distance to closest seed
  D = ((W.rowwise() - W.row(s)).array().square()).matrix().rowwise().sum();
  G *= 0;

  // greedily choose the remaining k-1 seeds
  for(int i = 1;i<k;i++)
  {
    // Find maximum in D
    D.maxCoeff(&s);
    S(i) = s;
    // distance to this seed
    Eigen::Matrix<double,Eigen::Dynamic,1> Ds =
      ((W.rowwise() - W.row(s)).array().square()).matrix().rowwise().sum();
    // Concatenation of D and Ds: DDs = [D Ds];
    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> DDs;
    // Make space for two columns
    DDs.resize(D.rows(),2);
    DDs.col(0) = D;
    DDs.col(1) = Ds;
    // Update D
    // get minimum of old D and distance to this seed, C == 1 if new distance
    // was smaller
    Eigen::Matrix<int,Eigen::Dynamic,1> C;
    igl::mat_min(DDs,2,D,C);
    G = (C.array() ==0).select(G,i);
  }


}
#endif
