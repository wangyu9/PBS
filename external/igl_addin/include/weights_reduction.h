#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Sparse>

Eigen::SparseMatrix<double> weights_pair_to_weights(const Eigen::MatrixXd& WV, const Eigen::MatrixXi& WI, const int cols);

void weights_pair_to_weights(const Eigen::MatrixXd& WV, const Eigen::MatrixXi& WI, const int cols, Eigen::SparseMatrix<double>& Wspr);

void weights_reduction(const Eigen::MatrixXd&W, const int numWeights, Eigen::MatrixXd& WV, Eigen::MatrixXi& WI);

void weights_reduction(const Eigen::MatrixXd&W, const int numWeights, Eigen::SparseMatrix<double>& Wspr);

Eigen::SparseMatrix<double> weights_reduction(const Eigen::MatrixXd&W, const int numWeights);