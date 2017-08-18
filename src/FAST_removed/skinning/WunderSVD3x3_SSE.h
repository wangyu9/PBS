#include <Eigen/Dense>

// Super fast 3x3 SVD according to http://pages.cs.wisc.edu/~sifakis/project_pages/svd.html
// This is SSE version of wunderSVD3x3 (see WunderSVD3x3.h) which works on 4 matrices at a time
// These four matrices are simply stacked in columns, the rest is the same as for wunderSVD3x3
template<typename T>
void wunderSVD3x3_SSE(const Eigen::Matrix<T, 3*4, 3>& A, Eigen::Matrix<T, 3*4, 3> &U, Eigen::Matrix<T, 3*4, 1> &S, Eigen::Matrix<T, 3*4, 3>&V);
