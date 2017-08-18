#ifndef IGL_ADDIN_POINTS_SET_H
#define IGL_ADDIN_POINTS_SET_H

#include <vector>
#include <Eigen/Dense>

class PointType{
public:
	double data[3];
	double& x;
	double& y;
	double& z;
	PointType(double xx, double yy, double zz): x(data[0]), y(data[1]), z(data[2])
	{
		data[0] = xx;
		data[1] = yy;
		data[2] = zz;
	}
	PointType& operator=(const PointType& other)
	{
		this->x = other.x;
		this->y = other.y;
		this->z = other.z;
		return *this;
	}
};

class PointsSet{
	std::vector<PointType> all;
public:
	void add(double *a)
	{
		this->add(a[0],a[1],a[2]);
	}
	void add(double x, double y, double z)
	{
		all.push_back(PointType(x,y,z));
	}
	void move(int index, double x, double y, double z)
	{
		all[index].data[0] += x;
		all[index].data[1] += y;
		all[index].data[2] += z;
	}
	void clear()
	{
		all.clear();
	}
	Eigen::MatrixXd toMat()
	{
		int n = all.size();
		Eigen::MatrixXd M(n,3);
		for (int i=0; i<n; i++)
		{
			M(i,0) = all[i].data[0];
			M(i,1) = all[i].data[1];
			M(i,2) = all[i].data[2];
		}
		return M;
	}
	void fromMat(const Eigen::MatrixXd& M)
	{
		assert(M.cols() == 3);
		clear();
		for (int i = 0; i < M.rows(); i++)
		{
			add(M(i,0), M(i,1), M(i,2));
		}
	}
};

#endif /*IGL_ADDIN_POINTS_SET_H*/