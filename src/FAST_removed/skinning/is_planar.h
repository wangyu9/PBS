#ifndef IS_PLANAR_H
#define IS_PLANAR_H

#include <Eigen/Dense>

// Return true if a mesh has constant value of 0 in z coordinate
// Inputs:
//
bool is_planar(const Eigen::MatrixXd & V);
//Implementation
bool is_planar(const Eigen::MatrixXd & V)
{
  if(V.size() == 0) return false;
  if(V.cols() == 2) return true;
  for(int i = 0;i<V.rows();i++)
  {
    if(V(i,2) != 0) return false;
  }
  return true;
}

#endif 
