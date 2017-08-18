#ifndef IGL_ADDIN_WRITE_H5MAT_H
#define IGL_ADDIN_WRITE_H5MAT_H

#include "igl/igl_inline.h"
#include <string>
#include <vector>
#include <Eigen/Core>

IGL_INLINE bool writeH5MAT(const std::string fname, const Eigen::MatrixXd& W);

#endif /*IGL_ADDIN_WRITE_H5MAT_H*/