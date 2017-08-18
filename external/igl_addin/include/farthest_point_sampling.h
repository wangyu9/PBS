#ifndef IGL_ADDIN_FARTHEST_POINT_SAMPLING
#define IGL_ADDIN_FARTHEST_POINT_SAMPLING

#include <vector>
#include <Eigen/Core>
#include <igl/igl_inline.h>
#include "types.h"
#include "handle.h" // to remove

// b0 is the ones before sampled.
// b1 is the sampled ones, including b0.
template <typename DerivedX>
bool farthest_point_sampling(const Eigen::PlainObjectBase<DerivedX> & VERTICES_MATRIX, const int num_handles, const Eigen::VectorXi& b0, Eigen::VectorXi& b1, const int seed_index_when_start_without_handle);

template <typename DerivedX>
bool farthest_point_sampling(const Eigen::PlainObjectBase<DerivedX> & VERTICES_MATRIX, const int seed_index_when_start_without_handle, const int num_handles, std::vector<Point3D>& all_handle_list);

//template <typename DerivedX>
//bool farthest_point_sampling(const Eigen::PlainObjectBase<DerivedX> * VERTICES_MATRIX, const int seed_index_when_start_without_handle, const int num_handles, std::vector<Point3D>& all_handle_list);
template <typename DerivedX>
void farthest_element_clustering(const Eigen::PlainObjectBase<DerivedX>& V, const Eigen::MatrixXi& TF, const int num_cluster, Eigen::VectorXi& new_rg);

#ifdef IGL_HEADER_ONLY
#include "farthest_point_sampling.cpp"
#endif

#endif /*IGL_ADDIN_FARTHEST_POINT_SAMPLING*/