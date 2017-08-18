#include <Eigen/Dense>

// Super fast 3x3 SVD according to http://pages.cs.wisc.edu/~sifakis/project_pages/svd.html
// This is AVX version of wunderSVD3x3 (see WunderSVD3x3.h) which works on 8 matrices at a time
// These four matrices are simply stacked in columns, the rest is the same as for wunderSVD3x3
template<typename T>
void wunderSVD3x3_AVX(const Eigen::Matrix<T, 3*8, 3>& A, Eigen::Matrix<T, 3*8, 3> &U, Eigen::Matrix<T, 3*8, 1> &S, Eigen::Matrix<T, 3*8, 3>&V);
