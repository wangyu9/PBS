#include "farthest_point_sampling.h"

#define INFINITE_DIST 1e50

// b0 is the ones before sampled.
// b1 is the sampled ones, including b0.
#include <igl/unique.h>
template <typename DerivedX>
bool farthest_point_sampling(const Eigen::PlainObjectBase<DerivedX> & V, const int num_handles, const Eigen::VectorXi& b0, Eigen::VectorXi& b1, const int seed_index_when_start_without_handle )
{
	assert(num_handles >= b0.rows());
	const size_t n0 = b0.rows();
	std::vector<Point3D> all_samples;
	all_samples.resize(n0);

	for (size_t i = 0; i < n0; i++)
	{
		all_samples[i] = Point3D( V(b0(i),0), V(b0(i),1), V(b0(i),2), b0(i) );
	}

	if (!farthest_point_sampling(V, seed_index_when_start_without_handle, num_handles, all_samples))
		return false;

	assert(num_handles + b0.rows() == all_samples.size());

	b1.resize(all_samples.size());

	for (size_t i = 0; i < all_samples.size(); i++)
	{
		b1(i) = all_samples[i].index;
	}

	/*Eigen::VectorXi ub1;
	igl::unique(b1, ub1);
	assert(ub1.rows() == b1.rows());*/

	return true;
}

template <typename DerivedX>
bool farthest_point_sampling(const Eigen::PlainObjectBase<DerivedX>& VERTICES_MATRIX, const int seed_index_when_start_without_handle, const int num_handles, std::vector<Point3D>& all_handle_list)
{

	assert(VERTICES_MATRIX.cols() == 3);

	int start_num = all_handle_list.size();
	std::vector<double> totalDistList(VERTICES_MATRIX.rows(), INFINITE_DIST);
	// Initialize the dist list
	for (int i = 0; i < totalDistList.size(); i++)
	{
		double x0;
		double y0;
		double z0;

		x0 = VERTICES_MATRIX(i, 0);//the vertices matrix is row major
		y0 = VERTICES_MATRIX(i, 1);
		z0 = VERTICES_MATRIX(i, 2);

		totalDistList[i] = INFINITE_DIST;

		for (int j = 0; j < all_handle_list.size(); j++)
		{
			double x;
			double y;
			double z;

			x = all_handle_list[j].x;
			y = all_handle_list[j].y;
			z = all_handle_list[j].z;

			double dx = x - x0;
			double dy = y - y0;
			double dz = z - z0;
			if (sqrt(dx*dx + dy*dy + dz*dz) < totalDistList[i])
				totalDistList[i] = sqrt(dx*dx + dy*dy + dz*dz);
		}

	}

	for (int j = 0; j < all_handle_list.size(); j++)
	{
		double x;
		double y;
		double z;

		x = all_handle_list[j].x;
		y = all_handle_list[j].y;
		z = all_handle_list[j].z;

		//printf("Pre-existing handles[%d] (%f,%f,%f)\n", j, x, y, z);
	}

	// Then the vertices that already in the handle list should be given a -INFINITE value
	for (int i = 0; i < all_handle_list.size(); i++)
	{
		if (all_handle_list[i].index >= 0 && all_handle_list[i].index < totalDistList.size())
		{
			totalDistList[all_handle_list[i].index] = -INFINITE_DIST;
		}
		else
		{
			printf("Warning: The existing handle points are not initialized correctly. Sample points may overlay with existing ones!\n");
		}
	}
	int new_added_index = seed_index_when_start_without_handle;
	std::vector<double> handleDistList;
	while (all_handle_list.size() < start_num + num_handles)
	{
		if (all_handle_list.size() == 0)//only in this case we use seed_index_when_start_without_handle
		{
			if (seed_index_when_start_without_handle < VERTICES_MATRIX.rows())
			{
				//The seed index is legal, start with it.
				new_added_index = seed_index_when_start_without_handle;
			}
			else
			{
				printf("Error: Seed index is out of the mesh vertices size.\n");
				return false;
			}
		}
		else
		{
			//typical case, do nothing
			double max_dist = -INFINITE_DIST;
			int max_index = -1;

			for (int i = 0; i < totalDistList.size(); i++)
			{
				if (totalDistList[i] == -INFINITE_DIST)
				{
					continue;//jump handles
				}

				if (totalDistList[i] > max_dist)
				{
					max_dist = totalDistList[i];
					max_index = i;
				}
			}

			if (max_index == -1)
			{
				printf("Warning: farthest_point_sampling(): too many points to be sampled (%d) in the function, exceeding the point set size()! Exit sampling.\n", start_num + num_handles);
				return false;
			}

			new_added_index = max_index;
		}
		
		//add the new handle to the list
		// printf("Sample handle[%d].\n", new_added_index);		
		double x0;
		double y0;
		double z0;

		x0 = VERTICES_MATRIX(new_added_index, 0);//the vertices matrix is row major
		y0 = VERTICES_MATRIX(new_added_index, 1);
		z0 = VERTICES_MATRIX(new_added_index, 2);

		totalDistList[new_added_index] = -INFINITE_DIST;
		all_handle_list.push_back(Point3D(x0, y0, z0, new_added_index));

		// then update the list

		for (int i = 0; i < totalDistList.size(); i++)
		{
			if (totalDistList[i] == -INFINITE_DIST)
			{
				continue;//jump handles
			}
			double x;
			double y;
			double z;

			x = VERTICES_MATRIX(i, 0);//the vertices matrix is row major
			y = VERTICES_MATRIX(i, 1);
			z = VERTICES_MATRIX(i, 2);

			double dx = x - x0;
			double dy = y - y0;
			double dz = z - z0;
			if (sqrt(dx*dx + dy*dy + dz*dz) < totalDistList[i])
				totalDistList[i] = sqrt(dx*dx + dy*dy + dz*dz);
		}

	}

	return true;
}


bool farthest_point_sampling0(const Eigen::MatrixXd * VERTICES_MATRIX, const int seed_index_when_start_without_handle, const int num_handles, std::vector<Point3D>& all_handle_list)
{
	int start_num = all_handle_list.size();
	std::vector<double> totalDistList(VERTICES_MATRIX->rows(), INFINITE_DIST);
	// Initialize the dist list
	for (int i = 0; i < totalDistList.size(); i++)
	{
		double x0;
		double y0;
		double z0;

		x0 = VERTICES_MATRIX->coeffRef(i, 0);//the vertices matrix is row major
		y0 = VERTICES_MATRIX->coeffRef(i, 1);
		z0 = VERTICES_MATRIX->coeffRef(i, 2);

		for (int j = 0; j < all_handle_list.size(); j++)
		{
			double x;
			double y;
			double z;

			x = VERTICES_MATRIX->coeffRef(j, 0);//the vertices matrix is row major 
			y = VERTICES_MATRIX->coeffRef(j, 1);
			z = VERTICES_MATRIX->coeffRef(j, 2);

			double dx = x - x0;
			double dy = y - y0;
			double dz = z - z0;
			if (sqrt(dx*dx + dy*dy + dz*dz) < totalDistList[i])
				totalDistList[i] = sqrt(dx*dx + dy*dy + dz*dz);
		}
	}

	// Then the vertices that already in the handle list should be given a INFINITE value
	for (int i = 0; i < all_handle_list.size(); i++)
	{
		if (all_handle_list[i].index >= 0 && all_handle_list[i].index < totalDistList.size())
		{
			totalDistList[all_handle_list[i].index] = -INFINITE_DIST;
		}
		else
		{
			printf("Warning: The existing handle points are not initialized correctly. Sample points may overlay with existing ones!\n");
		}
	}
	int new_added_index = seed_index_when_start_without_handle;
	std::vector<double> handleDistList;
	while (all_handle_list.size() < start_num + num_handles)
	{
		if (all_handle_list.size() == 0)//only in this case we use seed_index_when_start_without_handle
		{
			if (seed_index_when_start_without_handle < VERTICES_MATRIX->rows())
			{
				//The seed index is legal, start with it.
				new_added_index = seed_index_when_start_without_handle;
			}
			else
			{
				printf("Index is out of the mesh vertices size.\n");
				return false;
			}
		}
		else
		{
			//typical case, do nothing
		}
		double max_dist = -INFINITE_DIST;
		double temp_max_index = 0;

		assert(VERTICES_MATRIX->cols() == 3);
		double x0;
		double y0;
		double z0;

		x0 = VERTICES_MATRIX->coeffRef(new_added_index, 0);//the vertices matrix is row major
		y0 = VERTICES_MATRIX->coeffRef(new_added_index, 1);
		z0 = VERTICES_MATRIX->coeffRef(new_added_index, 2);

		for (int i = 0; i < totalDistList.size(); i++)
		{
			if (totalDistList[i] == -INFINITE_DIST)
			{
				continue;//jump handles
			}
			double x;
			double y;
			double z;

			x = VERTICES_MATRIX->coeffRef(i, 0);//the vertices matrix is row major
			y = VERTICES_MATRIX->coeffRef(i, 1);
			z = VERTICES_MATRIX->coeffRef(i, 2);

			double dx = x - x0;
			double dy = y - y0;
			double dz = z - z0;
			if (sqrt(dx*dx + dy*dy + dz*dz) < totalDistList[i])
				totalDistList[i] = sqrt(dx*dx + dy*dy + dz*dz);
			if (totalDistList[i] > max_dist)
			{
				max_dist = totalDistList[i];
				temp_max_index = i;
			}
		}
		//if(max_dist<min_max_dist)
		//{
		//	break;
		//}
		new_added_index = temp_max_index;
		printf("Sample handle[%d].\n", new_added_index);
		//add the new handle to the list
		double x;
		double y;
		double z;

		x = VERTICES_MATRIX->coeffRef(new_added_index, 0);//the vertices matrix is row major
		y = VERTICES_MATRIX->coeffRef(new_added_index, 1);
		z = VERTICES_MATRIX->coeffRef(new_added_index, 2);

		totalDistList[new_added_index] = -INFINITE_DIST;
		all_handle_list.push_back(Point3D(x, y, z, new_added_index));
		handleDistList.push_back(max_dist);
	}
	Eigen::MatrixXd handle_dist_matrix(handleDistList.size(), 1);
	for (int i = 0; i < handle_dist_matrix.rows(); i++)
	{
		handle_dist_matrix.coeffRef(i) = handleDistList[i];
	}
	//igl::writeDMAT("dist_list.dmat",handle_dist_matrix);
	return true;
}

bool farthest_point_sampling2(const Eigen::MatrixXd * VERTICES_MATRIX, const int seed_index_when_start_without_handle, const int num_handles, std::vector<Point3D>& all_handle_list)
{
	int start_num = all_handle_list.size();
	std::vector<double> totalDistList(VERTICES_MATRIX->rows(), INFINITE_DIST);
	// Initialize the dist list
	for (int i = 0; i < totalDistList.size(); i++)
	{
		double x0;
		double y0;
		double z0;

		x0 = VERTICES_MATRIX->coeffRef(i, 0);//the vertices matrix is row major
		y0 = VERTICES_MATRIX->coeffRef(i, 1);
		z0 = VERTICES_MATRIX->coeffRef(i, 2);

		for (int j = 0; j < all_handle_list.size(); j++)
		{
			double x;
			double y;
			double z;

			x = VERTICES_MATRIX->coeffRef(j, 0);//the vertices matrix is row major
			y = VERTICES_MATRIX->coeffRef(j, 1);
			z = VERTICES_MATRIX->coeffRef(j, 2);

			double dx = x - x0;
			double dy = y - y0;
			double dz = z - z0;
			if (sqrt(dx*dx + dy*dy + dz*dz) < totalDistList[i])
				totalDistList[i] = sqrt(dx*dx + dy*dy + dz*dz);
		}
	}

	// Then the vertices that already in the handle list should be given a INFINITE value
	for (int i = 0; i < all_handle_list.size(); i++)
	{
		if (all_handle_list[i].index >= 0 && all_handle_list[i].index < totalDistList.size())
		{
			totalDistList[all_handle_list[i].index] = -INFINITE_DIST;
		}
		else
		{
			printf("Warning: The existing handle points are not initialized correctly. Sample points may overlay with existing ones!\n");
		}
	}

	int new_added_index = seed_index_when_start_without_handle;
	std::vector<double> handleDistList;
	while (all_handle_list.size() < start_num + num_handles)
	{
		if (all_handle_list.size() == 0)//only in this case we use seed_index_when_start_without_handle
		{
			if (seed_index_when_start_without_handle < VERTICES_MATRIX->rows())
			{
				//The seed index is legal, start with it.
				new_added_index = seed_index_when_start_without_handle;
			}
			else
			{
				printf("Index is out of the mesh vertices size.\n");
				return false;
			}
		}
		else
		{
			//typical case
			if (all_handle_list.back().index >= 0 && all_handle_list.back().index < totalDistList.size())
			{
				new_added_index = all_handle_list.back().index;
			}
			else
			{
				printf("Warning: The last existing handle points are not initialized correctly.\n");
			}
		}
		double max_dist = -INFINITE_DIST;
		double temp_max_index = 0;

		assert(VERTICES_MATRIX->cols() == 3);
		double x0;
		double y0;
		double z0;

		x0 = VERTICES_MATRIX->coeffRef(new_added_index, 0);//the vertices matrix is row major
		y0 = VERTICES_MATRIX->coeffRef(new_added_index, 1);
		z0 = VERTICES_MATRIX->coeffRef(new_added_index, 2);

		for (int i = 0; i < totalDistList.size(); i++)
		{
			if (totalDistList[i] == -INFINITE_DIST)
			{
				continue;//jump handles
			}
			double x;
			double y;
			double z;

			x = VERTICES_MATRIX->coeffRef(i, 0);//the vertices matrix is row major
			y = VERTICES_MATRIX->coeffRef(i, 1);
			z = VERTICES_MATRIX->coeffRef(i, 2);

			double dx = x - x0;
			double dy = y - y0;
			double dz = z - z0;
			if (sqrt(dx*dx + dy*dy + dz*dz) < totalDistList[i])
				totalDistList[i] = sqrt(dx*dx + dy*dy + dz*dz);
			if (totalDistList[i] > max_dist)
			{
				max_dist = totalDistList[i];
				temp_max_index = i;
			}
		}
		//if(max_dist<min_max_dist)
		//{
		//	break;
		//}
		new_added_index = temp_max_index;
		printf("Sample handle[%d].\n", new_added_index);
		//add the new handle to the list
		double x;
		double y;
		double z;

		x = VERTICES_MATRIX->coeffRef(new_added_index, 0);//the vertices matrix is row major
		y = VERTICES_MATRIX->coeffRef(new_added_index, 1);
		z = VERTICES_MATRIX->coeffRef(new_added_index, 2);

		totalDistList[new_added_index] = -INFINITE_DIST;
		all_handle_list.push_back(Point3D(x, y, z, new_added_index));
		handleDistList.push_back(max_dist);
	}
	Eigen::MatrixXd handle_dist_matrix(handleDistList.size(), 1);
	for (int i = 0; i < handle_dist_matrix.rows(); i++)
	{
		handle_dist_matrix.coeffRef(i) = handleDistList[i];
	}
	//igl::writeDMAT("dist_list.dmat",handle_dist_matrix);
	return true;
}


template <typename DerivedX>
void farthest_element_clustering(const Eigen::PlainObjectBase<DerivedX>& V, const Eigen::MatrixXi& TF, const int num_cluster, Eigen::VectorXi& new_rg)
{
	std::cout << "farthest_element_clustering(" << num_cluster << ")" << std::endl;

	new_rg.resize(TF.rows());

	std::vector<Point3D> all_handle_list;
	farthest_point_sampling( V, 0, num_cluster, all_handle_list);

	Eigen::MatrixXd H(all_handle_list.size(), 3);
	for (int i = 0; i < all_handle_list.size(); i++)
	{
		H(i, 0) = all_handle_list[i].x;
		H(i, 1) = all_handle_list[i].y;
		H(i, 2) = all_handle_list[i].z;
	}

	Eigen::MatrixXd TF_POS(TF.rows(), 3);
	TF_POS.setZero();

	assert(TF.cols() == 3 || TF.cols() == 4);

	for (int i = 0; i < TF.rows(); i++)
	{
		for (int j = 0; j < TF.cols(); j++)
		{
			TF_POS.row(i) += V.row(TF(i, j));
		}
	}
	TF_POS *= 1.0 / TF.cols();

	for (int i = 0; i < TF_POS.rows(); i++)
	{
		double min_sq_dist = INFINITE_DIST;
		int min_sq_index = -1;
		for (int j = 0; j < H.rows(); j++)
		{
			double sq_dist = (TF_POS(i, 0) - H(j, 0))*(TF_POS(i, 0) - H(j, 0))
				+ (TF_POS(i, 1) - H(j, 1))*(TF_POS(i, 1) - H(j, 1))
				+ (TF_POS(i, 2) - H(j, 2))*(TF_POS(i, 2) - H(j, 2));
			if (sq_dist < min_sq_dist)
			{
				min_sq_dist = sq_dist;
				min_sq_index = j;
			}
		}
		new_rg(i) = min_sq_index;
	}
}


#ifdef IGL_STATIC_LIBRARY
// Explicit template specialization
template bool farthest_point_sampling<class Eigen::Matrix<double, -1, -1, 0, -1, -1> >(class Eigen::PlainObjectBase<class Eigen::Matrix<double, -1, -1, 0, -1, -1> > const &, int, class Eigen::Matrix<int, -1, 1, 0, -1, 1> const &, class Eigen::Matrix<int, -1, 1, 0, -1, 1> &, int);

template void farthest_element_clustering<class Eigen::Matrix<double, -1, -1, 0, -1, -1> >(class Eigen::PlainObjectBase<class Eigen::Matrix<double, -1, -1, 0, -1, -1> > const &, class Eigen::Matrix<int, -1, -1, 0, -1, -1> const &, int, class Eigen::Matrix<int, -1, 1, 0, -1, 1> &);

template void farthest_element_clustering<class Eigen::Matrix<double, -1, 3, 1, -1, 3> >(class Eigen::PlainObjectBase<class Eigen::Matrix<double, -1, 3, 1, -1, 3> > const &, class Eigen::Matrix<int, -1, -1, 0, -1, -1> const &, int, class Eigen::Matrix<int, -1, 1, 0, -1, 1> &);
#endif