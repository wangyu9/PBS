#include "weights_reduction.h"

#include <sort_weights.h>

void weights_reduction(const Eigen::MatrixXd&W, const int numWeights, Eigen::MatrixXd& WV, Eigen::MatrixXi& WI)
{
	const auto n = W.rows();
	const auto m = W.cols();
	const auto r = numWeights;

	WV.resize(n, r);
	WI.resize(n, r);

	Eigen::MatrixXd EW(n, 0);// no using extra weights

	sort_abs_weights(
		W, EW,
		numWeights,
		WV, WI);
}

Eigen::SparseMatrix<double> weights_pair_to_weights(const Eigen::MatrixXd& WV, const Eigen::MatrixXi& WI, const int cols)
{
	Eigen::SparseMatrix<double> Wspr;
	weights_pair_to_weights(WV, WI, cols, Wspr);
	return Wspr;
}

void weights_pair_to_weights(const Eigen::MatrixXd& WV, const Eigen::MatrixXi& WI, const int cols, Eigen::SparseMatrix<double>& Wspr)
{
	using namespace std;
	using namespace Eigen;

	assert(WV.rows() == WI.rows());
	assert(WV.cols() == WI.cols());

	Wspr.reserve(WV.size());

	vector<Triplet<double> > IJV;

	for (size_t r = 0; r < WV.rows(); r++)
		for (size_t c = 0; c < WV.cols(); c++)
			IJV.push_back(Triplet<double>(r, WI(r, c), WV(r, c)));

	Wspr.resize(WV.rows(), cols);
	Wspr.setFromTriplets(IJV.begin(), IJV.end());
}

void weights_reduction(const Eigen::MatrixXd&W, const int numWeights, Eigen::SparseMatrix<double>& Wspr)
{
	using namespace Eigen;
	
	MatrixXd WV;
	MatrixXi WI;

	weights_reduction(W, numWeights, WV, WI);

	weights_pair_to_weights(WV, WI, W.cols(), Wspr);
}

Eigen::SparseMatrix<double> weights_reduction(const Eigen::MatrixXd&W, const int numWeights)
{
	Eigen::SparseMatrix<double> Wspr;
	
	weights_reduction(W, numWeights, Wspr);

	return Wspr;
}