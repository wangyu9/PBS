#ifndef IGL_ADDIN_SORTROWS_H
#define IGL_ADDIN_SORTROWS_H

#include <igl/igl_inline.h> 

#include <vector>
#include <Eigen/Core>

// This is used for sorting the Data in *Each Row*

template <typename DerivedX, typename DerivedIX>
IGL_INLINE void sortrows(
	const Eigen::PlainObjectBase<DerivedX>& X,
	const int dim,
	const bool ascending,
	Eigen::PlainObjectBase<DerivedX>& Y,
	Eigen::PlainObjectBase<DerivedIX>& IX);

#endif IGL_ADDIN_SORTROWS_H