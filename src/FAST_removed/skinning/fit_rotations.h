#ifndef FIT_ROTATIONS_H
#define FIT_ROTATIONS_H

// FIT_ROTATIONS Given an input mesh and new positions find rotations for
// every covariance matrix in a stack of covariance matrices
// 
// Inputs:
//   S  nr*dim by dim stack of covariance matrices
// Outputs:
//   R  dim by dim * nr list of rotations
//
inline void fit_rotations(
  const Eigen::MatrixXf & S,
  Eigen::MatrixXf & R);

// FIT_ROTATIONS Given an input mesh and new positions find 2D rotations for
// every vertex that best maps its one ring to the new one ring
// 
// Inputs:
//   S  nr*dim by dim stack of covariance matrices, third column and every
//   third row will be ignored
// Outputs:
//   R  dim by dim * nr list of rotations, third row and third column of each
//   rotation will just be identity
//
inline void fit_rotations_planar(
  const Eigen::MatrixXf & S,
  Eigen::MatrixXf & R);

// Implementation
#include <igl/repmat.h>
#include <igl/verbose.h>
#include "polar_dec.h"
#include "polar_svd.h"
#include <iostream>
#include "wunder_polar_svd.h"

inline void fit_rotations(
  const Eigen::MatrixXf & S,
  Eigen::MatrixXf & R)
{
  const int dim = S.cols();
  const int nr = S.rows()/dim;
  assert(nr * dim == S.rows());

  // resize output
  R.resize(dim,dim*nr); // hopefully no op (should be already allocated)

  //std::cout<<"S=["<<std::endl<<S<<std::endl<<"];"<<std::endl;
  //MatrixXd si(dim,dim);
  Eigen::Matrix3f si; // = Eigen::Matrix3d::Identity();
  // loop over number of rotations we're computing
  for(int r = 0;r<nr;r++)
  {
    // build this covariance matrix
    for(int i = 0;i<dim;i++)
    {
      for(int j = 0;j<dim;j++)
      {
        si(i,j) = S(i*nr+r,j);
      }
    }
    Eigen::Matrix3f ri;
    Eigen::Matrix3f ti;
    //polar_dec(si,ri,ti);
    //polar_svd(si,ri,ti);
    wunder_polar_svd(si, ri);
    assert(ri.determinant() >= 0);

#ifndef FIT_ROTATIONS_ALLOW_FLIPS
    // Check for reflection
    if(ri.determinant() < 0)
    {
      // flip sign of last row
      ri.row(2) *= -1;
    }
    assert(ri.determinant() >= 0);
#endif  
    // Not sure why polar_dec computes transpose...
    R.block(0,r*dim,dim,dim) = ri.block(0,0,dim,dim).transpose();
  }
}

inline void fit_rotations_planar(
  const Eigen::MatrixXf & S,
  Eigen::MatrixXf & R)
{
  const int dim = S.cols();
  const int nr = S.rows()/dim;
  assert(nr * dim == S.rows());

  // resize output
  R.resize(dim,dim*nr); // hopefully no op (should be already allocated)

  Eigen::Matrix2f si; // = Eigen::Matrix3d::Identity();
  // loop over number of rotations we're computing
  for(int r = 0;r<nr;r++)
  {
    // build this covariance matrix
    for(int i = 0;i<2;i++)
    {
      for(int j = 0;j<2;j++)
      {
        si(i,j) = S(i*nr+r,j);
      }
    }
    Eigen::Matrix2f ri;
    Eigen::Matrix2f ti;
    polar_svd(si,ri,ti);
#ifndef FIT_ROTATIONS_ALLOW_FLIPS
    // Check for reflection
    if(ri.determinant() < 0)
    {
      // flip sign of last row
      ri.row(1) *= -1;
    }
    assert(ri.determinant() >= 0);
#endif  
    // Not sure why polar_dec computes transpose...
    R.block(0,r*dim,2,2) = ri.block(0,0,2,2).transpose();
    // Set remaining part to identity
    R(0,r*3+2) = 0;
    R(1,r*3+2) = 0;
    R(2,r*3+0) = 0;
    R(2,r*3+1) = 0;
    R(2,r*3+2) = 1;
  }
}

#ifndef __APPLE__

void fit_rotations_SSE(
  const Eigen::MatrixXf & S,
  Eigen::MatrixXf & R)
{
  const int cStep = 4;

  assert(S.cols() == 3);
  const int dim = 3; //S.cols();
  const int nr = S.rows()/dim;  
  assert(nr * dim == S.rows());

  // resize output
  R.resize(dim,dim*nr); // hopefully no op (should be already allocated)

  Eigen::Matrix<float, 3*cStep, 3> siBig;
  // using SSE decompose cStep matrices at a time:
  int r = 0;
  for( ; r<nr; r+=cStep)
  {
    int numMats = cStep;
    if (r + cStep >= nr) numMats = nr - r;
    // build siBig:
    for (int k=0; k<numMats; k++)
    {
      for(int i = 0;i<dim;i++)
      {
        for(int j = 0;j<dim;j++)
        {
          siBig(i + 3*k, j) = S(i*nr + r + k, j);
        }
      }
    }
    Eigen::Matrix<float, 3*cStep, 3> ri;
    wunder_polar_svd_SSE(siBig, ri);    

    for (int k=0; k<cStep; k++)
      assert(ri.block(3*k, 0, 3, 3).determinant() >= 0);

    // Not sure why polar_dec computes transpose...
    for (int k=0; k<numMats; k++)
    {
      R.block(0, (r + k)*dim, dim, dim) = ri.block(3*k, 0, dim, dim).transpose();
    }    
  }
}

void fit_rotations_AVX(
  const Eigen::MatrixXf & S,
  Eigen::MatrixXf & R)
{
  const int cStep = 8;

  assert(S.cols() == 3);
  const int dim = 3; //S.cols();
  const int nr = S.rows()/dim;  
  assert(nr * dim == S.rows());

  // resize output
  R.resize(dim,dim*nr); // hopefully no op (should be already allocated)

  Eigen::Matrix<float, 3*cStep, 3> siBig;
  // using SSE decompose cStep matrices at a time:
  int r = 0;
  for( ; r<nr; r+=cStep)
  {
    int numMats = cStep;
    if (r + cStep >= nr) numMats = nr - r;
    // build siBig:
    for (int k=0; k<numMats; k++)
    {
      for(int i = 0;i<dim;i++)
      {
        for(int j = 0;j<dim;j++)
        {
          siBig(i + 3*k, j) = S(i*nr + r + k, j);
        }
      }
    }
    Eigen::Matrix<float, 3*cStep, 3> ri;
    wunder_polar_svd_AVX(siBig, ri);    

    for (int k=0; k<cStep; k++)
      assert(ri.block(3*k, 0, 3, 3).determinant() >= 0);

    // Not sure why polar_dec computes transpose...
    for (int k=0; k<numMats; k++)
    {
      R.block(0, (r + k)*dim, dim, dim) = ri.block(3*k, 0, dim, dim).transpose();
    }    
  }
}
#endif


#endif
