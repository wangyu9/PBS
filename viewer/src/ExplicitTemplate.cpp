//#ifdef IGL_STATIC_LIBRARY
// Explicit template specialization

//#include <igl/edge_lengths.h>
//template void igl::edge_lengths<class Eigen::Matrix<double, -1, 3, 1, -1, 3>, class Eigen::Matrix<unsigned int, -1, -1, 1, -1, -1>, class Eigen::Matrix<double, -1, 3, 1, -1, 3> >(class Eigen::PlainObjectBase<class Eigen::Matrix<double, -1, 3, 1, -1, 3> > const &, class Eigen::PlainObjectBase<class Eigen::Matrix<unsigned int, -1, -1, 1, -1, -1> > const &, class Eigen::PlainObjectBase<class Eigen::Matrix<double, -1, 3, 1, -1, 3> > &);

//wangyu
#include <igl/adjacency_list.h>
template void igl::adjacency_list<class Eigen::Matrix<unsigned int, -1, -1, 1, -1, -1>, unsigned int>(class Eigen::PlainObjectBase<class Eigen::Matrix<unsigned int, -1, -1, 1, -1, -1> > const &, class std::vector<class std::vector<unsigned int, class std::allocator<unsigned int> >, class std::allocator<class std::vector<unsigned int, class std::allocator<unsigned int> > > > &, bool);


//#endif

