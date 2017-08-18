#ifndef IGL_ADDIN_REINFORCE_COORDINATE
#define IGL_ADDIN_REINFORCE_COORDINATE

#include <Eigen/Dense>

void reproject_coordinate(Eigen::MatrixXd & W_value, Eigen::MatrixXi & W_index, Eigen::MatrixXd & pre_W_value, Eigen::MatrixXi & pre_W_index, Eigen::MatrixXd & V, Eigen::MatrixXd & H, int dim);

void reproject_coordinate_individually(Eigen::MatrixXd & W_value, const Eigen::MatrixXi & W_index, const Eigen::MatrixXd & V, const Eigen::MatrixXd & H, const int dim, const double row_sum);

#ifdef IGL_HEADER_ONLY
#include "reproject_coordinate.cpp"
#endif

#endif IGL_ADDIN_REINFORCE_COORDINATE