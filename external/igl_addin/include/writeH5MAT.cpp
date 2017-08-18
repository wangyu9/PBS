#include "writeH5MAT.h"

#include "eigen3-hdf5.hpp"

// https://github.com/garrison/eigen3-hdf5

bool writeH5MAT(const std::string fname, const Eigen::MatrixXd& W)
{

	H5::H5File file(fname.c_str(), H5F_ACC_TRUNC);
	EigenHDF5::save(file, "H5MATRIXDENSE", W.transpose());

	return true;
}