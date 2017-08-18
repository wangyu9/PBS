#ifndef IGL_ADDIN_READ_H5MAT_H
#define IGL_ADDIN_READ_H5MAT_H

#include "igl/igl_inline.h"
#include <string>
#include <vector>
#include <Eigen/Core>

IGL_INLINE bool readH5MAT(const std::string fname, Eigen::MatrixXd& W);

#endif /*IGL_ADDIN_READ_H5MAT_H*/