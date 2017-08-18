#ifndef IGL_ADDIN_UNIQUE_PLUS_H
#define IGL_ADDIN_UNIQUE_PLUS_H

#include <igl/igl_inline.h> 

#include <Eigen/Core>

// Inputs:
// dim: 0 not implemented yet, 1 'cols', 2 'rows'

// Act like matlab's [C,IA,IC] = unique(X,'rows')
//
// Templates:
//   DerivedA derived scalar type, e.g. MatrixXi or MatrixXd
//   DerivedIA derived integer type, e.g. MatrixXi
//   DerivedIC derived integer type, e.g. MatrixXi
// Inputs:
//   A  m by n matrix whose entries are to unique'd according to rows
// Outputs:
//   C  #C vector of unique rows in A
//   IA  #C index vector so that C = A(IA,:);
//   IC  #A index vector so that A = C(IC,:);
template <typename DerivedA, typename DerivedIA, typename DerivedIC>
IGL_INLINE void unique_rows_plus(
	const Eigen::PlainObjectBase<DerivedA>& A,
	Eigen::PlainObjectBase<DerivedA>& C,
	Eigen::PlainObjectBase<DerivedIA>& IA,
	Eigen::PlainObjectBase<DerivedIC>& IC);

#ifdef IGL_HEADER_ONLY
	#include "unique_plus.cpp"
#endif

#endif /*IGL_ADDIN_UNIQUE_PLUS_H*/
