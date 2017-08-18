#ifndef IGL_ADDIN_EIGEN_HELPER_H
#define IGL_ADDIN_EIGEN_HELPER_H


#include <Eigen/Core>

inline Eigen::MatrixXd Rows_Average(const Eigen::MatrixXd& A)
{
	Eigen::MatrixXd rows_avg = Eigen::MatrixXd::Zero(1, A.cols());

	for (int i = 0; i < A.rows(); i++)
	{
		rows_avg = rows_avg + A.row(i);
	}

	if (A.rows()>0)
	{
		rows_avg = rows_avg * 1.0 / A.rows();
	}

	return rows_avg;
}

//template <typename DerivedA>
inline Eigen::MatrixXd Rows_of(const Eigen::MatrixXd & A, const Eigen::VectorXi & b)
{
	//assert(b.cols() == 1);
	Eigen::MatrixXd R;
	R.resize(b.rows(), A.cols());
	for (int i = 0; i < b.rows(); i++)
	{
		R.row(i) = A.row(b(i));
	}
	return R;
}


inline Eigen::VectorXi NaturalSeq(const int m, const int start = 0)
{// it cannot return reference type, which will be deconstructed when function returns.
	Eigen::VectorXi N(m);
	for (int i = 0; i < m; i++)
	{
		N(i) = start + i; // each handle to be in a unique group.
	}
	return N;
}

#include <vector>
inline void complementary(const Eigen::VectorXi& indices, const int n, Eigen::VectorXi& result)
{
	int c = n - indices.rows();
	assert(c >= 0);

	result.resize(c);

	std::vector<bool> unknown_mask;
	unknown_mask.resize(n, true);
	for (int i = 0; i < indices.rows(); i++)
	{
		assert(indices(i) < n);
		unknown_mask[indices(i)] = false;
	}

	int u = 0;

	for (int i = 0; i < n; i++)
	{
		if (unknown_mask[i])
		{
			result(u) = i;
			u++;
		}
	}

	assert(u == c);
}

inline Eigen::VectorXi complementary(const Eigen::VectorXi& indices, const int n)
{
	Eigen::VectorXi r;
	complementary(indices, n, r);
	return r;
}

#endif /*IGL_ADDIN_EIGEN_HELPER_H*/