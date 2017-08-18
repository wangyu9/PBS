#include "move_point_onto_mesh.h"

void attach_all_handles_to_nearest_vertices(const Eigen::MatrixXd * vertices_to_search, std::vector<Point3D>& all_handle_list)
{
	for (int i = 0; i < all_handle_list.size(); i++)
	{
		double sq_min_dist = 1e15;
		int min_index = -1;
		for (int j = 0; j < vertices_to_search->rows(); j++)
		{
			double dx = all_handle_list[i].x - (*vertices_to_search)(j, 0);
			double dy = all_handle_list[i].y - (*vertices_to_search)(j, 1);
			double dz = all_handle_list[i].z - (*vertices_to_search)(j, 2);
			double sq_temp_dist = dx*dx + dy*dy + dz*dz;
			if (sq_temp_dist < sq_min_dist)
			{
				min_index = j;
				sq_min_dist = sq_temp_dist;
			}
		}
		all_handle_list[i].index = min_index;
		if (min_index >= 0)
		{
			all_handle_list[i].x = (*vertices_to_search)(min_index, 0);
			all_handle_list[i].y = (*vertices_to_search)(min_index, 1);
			all_handle_list[i].z = (*vertices_to_search)(min_index, 2);
		}
	}
}

