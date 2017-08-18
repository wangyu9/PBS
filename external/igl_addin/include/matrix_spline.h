#ifndef IGL_ADDIN_MATRIX_SPLINE_H
#define IGL_ADDIN_MATRIX_SPLINE_H

// reference: http://www.geos.ed.ac.uk/~yliu23/docs/lect_spline.pdf equation(3.7)

#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>

class CubicCoefficients{
public:
	double a;
	double b;
	double c;
	double d;
};

class DataNode{
public:
	double x;
	//Eigen::MatrixXd M;
	DataNode(const double xx) : x(xx){}//, const Eigen::MatrixXd& MM): x(xx), M(MM){}
};

class CubicSpline{
public:
	CubicSpline() :min_x(0.), max_x(0.), has_set_coeffs(false){}
	std::vector<DataNode> nodes;
	std::vector<Eigen::MatrixXd> coeffs;
	std::vector<Eigen::MatrixXd> subcoeffs;
	double min_x;
	double max_x;
	bool has_set_coeffs;
	double get_start() const
	{
		return min_x;
	}
	double get_end() const 
	{
		return max_x;
	}
	int get_index(const double xx) const
	{
		for (int i = nodes.size() - 2; i >= 0; i--) // important, the number of duration is less than nodes number.
		{
			if (xx >= nodes[i].x)
				return i;
		}
	}
	void clear()
	{
		CubicSpline();
		nodes.clear();
		coeffs.clear();
		subcoeffs.clear();
	}
	void append(const double xx)
	{
		assert(nodes.size()==0 || xx >= nodes.back().x);// Only supports increasing x right now.

		if (nodes.size() == 0)
			min_x = xx;
		max_x = xx;

		nodes.push_back(DataNode(xx));
	}
	void solve_coeffs()
	{
		int n = nodes.size();
		if (n == 0)
			return;

		has_set_coeffs = true;

		Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n, n);
		Eigen::MatrixXd B = Eigen::MatrixXd::Zero(n, n);
		A(0, 0) = 1;
		A(n - 1, n - 1) = 1;

		// We have A * f" = B * f
		// so f" = A\B * f = D2 * f;
		for (int i = 1; i < n-1; i++)
		{
			A(i, i - 1) = (nodes[i].x - nodes[i - 1].x) / 6.;
			A(i, i) = (nodes[i + 1].x - nodes[i - 1].x) / 3.;
			A(i, i + 1) = (nodes[i + 1].x - nodes[i].x) / 6.;

			B(i, i - 1) = 1. / (nodes[i].x - nodes[i - 1].x);
			B(i, i) = -1./(nodes[i].x - nodes[i - 1].x) - 1./(nodes[i + 1].x - nodes[i].x);
			B(i, i + 1) = 1. / (nodes[i+1].x - nodes[i].x);
		}

		std::cout << "CubicSpline A:\n" << A << std::endl;
		std::cout << "CubicSpline B:\n" << B << std::endl;

		Eigen::MatrixXd D2 = A.colPivHouseholderQr().solve(B);

		std::cout << "CubicSpline D2 Matrix:\n" << D2 << std::endl;

		coeffs.clear();
		subcoeffs.clear();

		// We have X * (d,c,b,a)^T = D2;

		for (int i = 0; i < n - 1; i++)// the number of coeffs is n-1
		{
			Eigen::MatrixXd X = Eigen::MatrixXd::Zero(4,4);

			const int left = i;
			const int right = i + 1;

			double xkl[4] = { 1. }; // this is x^k
			double xkr[4] = { 1. };
			for (int k = 1; k < 4; k++)
			{
				xkl[k] = xkl[k-1] * nodes[left].x;
				xkr[k] = xkr[k-1] * nodes[right].x;
			}

			for (int k = 0; k < 4; k++)
			{
				X(0, k) = xkl[k]; // this is d + x*c + x^2 *b + x^3*a; on the left side
				X(1, k) = xkr[k]; // the same on the right side
				if (k<=1)
				{
					X(2, k) = 0.;
					X(3, k) = 0.;
				}
				else
				{
					X(2, k) = k * (k-1) * xkl[k - 2]; // this is 0 * d + 0 * c + 2 * b + 6x *d
					X(3, k) = k * (k-1) * xkr[k - 2];
				}			
			}

			//Eigen::VectorXd b = Eigen::VectorXd::Ones(4); // the b corresponds to (f_left, f_right, f"_left, f"_right)

			Eigen::MatrixXd R = X.inverse(); //X.colPivHouseholderQr().solve(b);

			std::cout << "CubicSpline R Matrix:\n" << R << std::endl;

			subcoeffs.push_back(R);

			Eigen::MatrixXd Coeffs = Eigen::MatrixXd::Zero(4, n);// this is  the matrix such that (d,c,b,a)^T = Coeffs * f; f is the vector of all

			Coeffs = R.col(2) * D2.row(left) + R.col(3) * D2.row(right);
			Coeffs.col(left) += R.col(0);
			Coeffs.col(right) += R.col(1);

			coeffs.push_back(Coeffs);
		}
	}

	void print_coeffs()
	{
		std::cout << "CubicSpline\n" << std::endl;
		for (int i = 0; i < coeffs.size(); i++)
		{
			std::cout << "Coeffs[" << i << "] :\n" << coeffs[i] << std::endl;
		}
	}
};


class MatrixCubicSpline : public CubicSpline{
public:
	std::vector<Eigen::MatrixXd> Ds;
	std::vector<Eigen::MatrixXd> Cs;
	std::vector<Eigen::MatrixXd> Bs;
	std::vector<Eigen::MatrixXd> As;

	std::vector<Eigen::MatrixXd> Mats;

	bool has_build_matrix;

	MatrixCubicSpline():CubicSpline(),has_build_matrix(false){}

	void clear()
	{
		CubicSpline::clear();

		MatrixCubicSpline();
		Ds.clear();
		Cs.clear();
		Bs.clear();
		As.clear();

		Mats.clear();
	}
	bool append_matrix(const Eigen::MatrixXd& nM) // here should guarantees all matrices have the same sizes.
	{
		Mats.push_back(nM);

		return true;
	}

	void build_matrix_coeffs()
	{
		assert(has_set_coeffs);
		if (!has_set_coeffs)
		{
			return;
		}
		assert(Mats.size()==nodes.size()&&Mats.size()>0);

		int n = Mats.size();

		int r = Mats[0].rows();
		int c = Mats[0].cols();

		for (int i = 1; i < Mats.size(); i++)
		{
			assert(Mats[i].rows() == r);
			assert(Mats[i].cols() == c);
		}

		has_build_matrix = true;

		// Init

		Ds.clear();
		Cs.clear();
		Bs.clear();
		As.clear();

		for (int t = 0; t < n - 1; t++)
		{
			Ds.push_back(Eigen::MatrixXd::Zero(r, c));
			Cs.push_back(Eigen::MatrixXd::Zero(r, c));
			Bs.push_back(Eigen::MatrixXd::Zero(r, c));
			As.push_back(Eigen::MatrixXd::Zero(r, c));
		}

		// Set each entries

		for (int i = 0; i < r; i++)
		{
			for (int j = 0; j < c; j++)
			{
				Eigen::VectorXd f(n);
				for (int k = 0; k < n; k++)
				{
					f(k) = Mats[k](i, j);
				}

				for (int t = 0; t < n - 1; t++)
				{
					Eigen::MatrixXd Coeffs_t = coeffs[t] * f;
					assert(Coeffs_t.rows() == 4 && Coeffs_t.cols() == 1);

					Ds[t](i, j) = Coeffs_t(0);
					Cs[t](i, j) = Coeffs_t(1);
					Bs[t](i, j) = Coeffs_t(2);
					As[t](i, j) = Coeffs_t(3);
				}
			}
		}

	}

	Eigen::MatrixXd interpolate(double xx) const
	{
		assert(has_build_matrix);

		xx = (xx < get_start()) ? get_start() : xx;
		xx = (xx > get_end()) ? get_end() : xx;

		int index = get_index(xx);

		int r = Mats[0].rows();
		int c = Mats[0].cols();

		Eigen::MatrixXd result = Eigen::MatrixXd::Zero(r,c);

		result = Ds[index] + xx * ( Cs[index] + xx * ( Bs[index] + xx * As[index] ) );

		return result;
	}

	Eigen::MatrixXd get_interpolate_matrix(double xx, Eigen::MatrixXd& DD, Eigen::MatrixXd& CC, Eigen::MatrixXd& BB, Eigen::MatrixXd& AA) const
	{
		assert(has_build_matrix);

		xx = (xx < get_start()) ? get_start() : xx;
		xx = (xx > get_end()) ? get_end() : xx;

		int index = get_index(xx);

		int r = Mats[0].rows();
		int c = Mats[0].cols();

		Eigen::MatrixXd result = Eigen::MatrixXd::Zero(r, c);

		result = Ds[index] + xx * (Cs[index] + xx * (Bs[index] + xx * As[index]));
		
		DD = Ds[index];
		CC = Cs[index];
		BB = Bs[index];
		AA = As[index];

		return result;
	}
};

#endif /*IGL_ADDIN_MATRIX_SPLINE_H*/