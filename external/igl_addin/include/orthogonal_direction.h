#ifndef IGL_ADDIN_ORTHOGONAL_DIRECTION_h
#define IGL_ADDIN_ORTHOGONAL_DIRECTION_h

#include <Eigen/Dense>

// orthogonal3d:
// Input: a 3d vector n
// Output: two unit 3d vector that are orthogonal to n and also with each other
// (such that orthogonal_v1.cross(orthogonal_v2) == n.normalized();
inline void orthogonal3d(const Eigen::Vector3d& n, Eigen::Vector3d& orthogonal_v1, Eigen::Vector3d& orthogonal_v2)
{
	const Eigen::Vector3d normal_dir = n.normalized();
	double a = normal_dir(0);
	double b = normal_dir(1);
	double c = normal_dir(2);
	Eigen::Vector3d t0;
	Eigen::Vector3d b0;
	if (abs(a)>=abs(b))
	{
		if (abs(c)>=abs(a))
		{
			// |c| is the max
			t0 = Eigen::Vector3d(1,0,0);
			b0 = Eigen::Vector3d(0,-1,0);
		} 
		else
		{
			// |a| is the max
			t0 = Eigen::Vector3d(0,-1,0);
			b0 = Eigen::Vector3d(0,0,-1);
		}
	}
	else
	{
		if (abs(c)>=abs(b))
		{
			// |c| is the max
			t0 = Eigen::Vector3d(1,0,0);
			b0 = Eigen::Vector3d(0,-1,0);
		} 
		else
		{
			// |b| is the max
			t0 = Eigen::Vector3d(1,0,0);
			b0 = Eigen::Vector3d(0,0,-1);
		}
	}
	//t0 = Eigen::Vector3d(1,-1,0);
	//b0 = Eigen::Vector3d(5,6,0);

	//Eigen::Vector3d 
	orthogonal_v1 = normal_dir.cross(t0.cross(normal_dir)).normalized();
	//Eigen::Vector3d orthogonal_v2 = normal_dir.cross(b0.cross(normal_dir)).normalized();
	//Eigen::Vector3d 
	orthogonal_v2 = - normal_dir.cross(orthogonal_v1).normalized();
	if ( normal_dir.dot(orthogonal_v1.cross(orthogonal_v2))>0 )
	{
		;// nothing
	} 
	else
	{
		orthogonal_v2 = -orthogonal_v2;
	}

	// double check
	double prod = normal_dir.dot((orthogonal_v1.cross(orthogonal_v2)).normalized());
	if (prod<0.9)
	{
		printf("Warning: initializing tangent vectors incorrectly! with inner product: %f.\n", prod);
	}
};


// orghogonal2d:
// Input: n is a *2D* vector.
// Output: orthogonal_v is a 2D unit vector that is orthogonal to n.
inline void orthogonal2d(const Eigen::Vector2d& n, Eigen::Vector2d& orthogonal_v)
{
	// making use of the fact rotating the vector (a,b,0) 90 degrees around (0,0,1) will lead to vector (-b,a,0)

	orthogonal_v(0) = - n(1);
	orthogonal_v(1) = n(0);
	orthogonal_v.normalize();
}



#endif /*IGL_ADDIN_ORTHOGONAL_DIRECTION_h*/