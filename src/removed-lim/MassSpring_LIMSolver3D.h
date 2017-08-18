// Copyright 2013 - Christian Schüller 2013, schuellc@inf.ethz.ch
// Interactive Geometry Lab - ETH Zurich

#pragma once

#ifndef MASS_SPRING_LIM_SOLVER_3D_H
#define MASS_SPRING_LIM_SOLVER_3D_H

#include "LIMSolver3D.h"
#include <vector>

class spring{
public:
	int start_index;
	int end_index;
	double rest_len;
	double k;
};

class MassSpring_LIMSolver3D : public LIMSolver3D
{
public:

  MassSpring_LIMSolver3D();
  ~MassSpring_LIMSolver3D();

private:
	std::vector<spring> spring_list;


  Eigen::SparseMatrix<double> Identity;

  void debugOutput(std::stringstream& info);
  void prepareProblemData(std::vector<int>& hessRowIdx, std::vector<int>& hessColIdx);
  double computeFunction(const Eigen::Matrix<double,Eigen::Dynamic,1>& x);
  void computeGradient(const Eigen::Matrix<double,Eigen::Dynamic,1>& x, Eigen::Matrix<double,Eigen::Dynamic,1>& grad);
  void computeHessian(const Eigen::Matrix<double,Eigen::Dynamic,1>& x, const Eigen::Matrix<double*,Eigen::Dynamic,1>& hess);

  bool has_inited;
  void setInitPoseAsRestPose();
  void setCurrentPoseAsRestPose();
};

#endif