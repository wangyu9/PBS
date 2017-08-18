#include "readH5MAT.h"

#include "eigen3-hdf5.hpp"

// https://github.com/garrison/eigen3-hdf5

bool readH5MAT(const std::string fname, Eigen::MatrixXd& W)
{

	Eigen::MatrixXd W_t;

	H5::H5File file(fname.c_str(), H5F_ACC_RDONLY);
	EigenHDF5::load(file, "H5MATRIXDENSE", W_t);

	W = W_t.transpose();

	return true;
}