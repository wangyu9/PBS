#ifndef IGL_ADDIN_MOVE_POINT_ONTO_MESH
#define IGL_ADDIN_MOVE_POINT_ONTO_MESH

#include <Eigen/Core>
#include <vector>
#include "handle.h"

void attach_all_handles_to_nearest_vertices(const PointMatrixType * vertices_to_search, std::vector<Point3D>& all_handle_list);


#endif /*IGL_ADDIN_MOVE_POINT_ONTO_MESH*/