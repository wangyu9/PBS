#ifndef IGL_ADDIN_SKELETON_SAMPLING_H
#define IGL_ADDIN_SKELETON_SAMPLING_H

#define HALF_SQRT3 0.86602540378

//#include <Eigen/Dense>
#include "orthogonal_direction.h"

// num_seg is the number of segment of sampled bones, 
// the output will have num_seg segments, (num_seg+1) ends.

inline void volumetric_skeleton_sampling(
	const Eigen::VectorXd& start,
	const Eigen::VectorXd& end,
	//const Eigen::MatrixXd& twoEnd, 
	const int dim,
	const int num_seg,
	const int sample_per_seg,
	const int ring,
	const double radius,
	Eigen::MatrixXd& ske,
	bool append_twoends = false,
	bool cylinder_or_ellipsoid = true)// true for cylinder
{
	//assert(twoEnd.rows()==2&&twoEnd.cols()==3);
	assert(start.rows()==3||end.rows()==3);
	assert(num_seg>=1);
	assert(ring>0);// decides to allow ring equals to 0
	assert(sample_per_seg>=1);

	Eigen::MatrixXd temp_ske;

	if (dim==3)
	{
		assert(start.rows()==3&&end.rows()==3);
		Eigen::Vector3d n = end - start;
		Eigen::Vector3d v1; 
		Eigen::Vector3d v2;
		orthogonal3d(n,v1,v2);
		temp_ske.resize(sample_per_seg*(num_seg+1)*ring,3);
		for(int k=0; k<ring; k++)
		{ 
			for (int i=0; i<=num_seg; i++)
			{
				double r = (k + 1) * radius / ring;
				if (!cylinder_or_ellipsoid)
					r *= (sin(i*M_PI/num_seg)*0.99+0.01);// to avoid striking to a single point
				Eigen::Vector3d c = start + n*(1.0*i)/num_seg;
				//ske(i*3+0) = c + radius*(HALF_SQRT3*v1-0.5*v2);
				//ske(i*3+1) = c + radius*(v2);
				//ske(i*3+2) = c + radius*(-HALF_SQRT3*v1-0.5*v2);
				for (int j=0; j<sample_per_seg; j++)
				{
					double theta = 2*M_PI*j/sample_per_seg;
					temp_ske.row( k*sample_per_seg*(num_seg+1)+(i*sample_per_seg+j) ) = ( c + r*(v1*cos(theta)+v2*sin(theta)) ).transpose();
				}
			
			}
		}
	}
	else
	{
		assert(dim==2);
		assert(start.rows()==3&&end.rows()==3);
		Eigen::Vector3d n = end- start;
		n(2) = 0;
		Eigen::Vector2d n2d;
		n2d(0) = n(0);
		n2d(1) = n(1);
		Eigen::Vector2d v2d; 
		orthogonal2d(n2d,v2d);
		Eigen::Vector3d v;
		v(0) = v2d(0);
		v(1) = v2d(1);
		v(2) = 0.;
		temp_ske.resize(2*(num_seg+1)*ring,3);
		for(int k=0; k<ring; k++)
		{
			for (int i=0; i<=num_seg; i++)
			{
				double r = (k + 1) * radius / ring;
				if (!cylinder_or_ellipsoid)
					r *= (sin(i*M_PI / num_seg)*0.99 + 0.01);// to avoid striking to a single point
				Eigen::Vector3d c = start + n*(1.0*i)/num_seg;
				c(2) = 0;
				for (int j=0; j<2; j++)
				{
					double sign = (j==0)?1.0:-1.0;
					temp_ske.row( k*2*(num_seg+1)+(i*2+j) ) = ( c + v*r*(sign) ).transpose();
				}

			}
		}
	}

	if (append_twoends)
	{
		if (abs(radius)>10e-6)
		{
			ske.resize(temp_ske.rows() + 2, 3);
			ske.row(0) = start.transpose();
			ske.row(1) = end.transpose();
			ske.block(2, 0, temp_ske.rows(), 3) = temp_ske;
		} 
		else
		{
			ske = temp_ske;//do not need to warning this. printf("Warning: ignoring append_twoends due to small radius(%d).", );
			int nn = ske.rows();
			// swap the two because twoends are always in the beginning.
			ske.row(1) = temp_ske.row(nn-1);
			ske.row(nn - 1) = temp_ske.row(1);
		}
	} 
	else
	{
		ske = temp_ske;
	}
}


#endif /*IGL_ADDIN_SKELETON_SAMPLING_H*/