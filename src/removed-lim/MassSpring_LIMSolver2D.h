// Copyright 2013 - Christian Schüller 2013, schuellc@inf.ethz.ch
// Interactive Geometry Lab - ETH Zurich

#pragma once

#ifndef MASS_SPRING_LIM_SOLVER_2D_H
#define MASS_SPRING_LIM_SOLVER_2D_H

#include "LIMSolver2D.h"

class MassSpring_LIMSolver2D : public LIMSolver2D
{
public:

  MassSpring_LIMSolver2D();
  ~MassSpring_LIMSolver2D();

private:

  Eigen::SparseMatrix<double> Identity;
  
  void debugOutput(std::stringstream& info);
  void prepareProblemData(std::vector<int>& hessRowIdx, std::vector<int>& hessColIdx);
  double computeFunction(const Eigen::Matrix<double,Eigen::Dynamic,1>& x);
  void computeGradient(const Eigen::Matrix<double,Eigen::Dynamic,1>& x, Eigen::Matrix<double,Eigen::Dynamic,1>& grad);
  void computeHessian(const Eigen::Matrix<double,Eigen::Dynamic,1>& x, const Eigen::Matrix<double*,Eigen::Dynamic,1>& hess);
};

#endif