// This file is part of libigl, a simple c++ geometry processing library.
// 
// Copyright (C) 2013 Alec Jacobson <alecjacobson@gmail.com>
// 
// This Source Code Form is subject to the terms of the Mozilla Public License 
// v. 2.0. If a copy of the MPL was not distributed with this file, You can 
// obtain one at http://mozilla.org/MPL/2.0/.
#ifndef IGL_ADDIN_WRITEHANDLE_H
#define IGL_ADDIN_WRITEHANDLE_H
#include <igl/igl_inline.h>

#include <string>
#include <vector>
#include <Eigen/Core>

  // save a tetrahedral volume mesh to a .mesh file
  //
  // Templates:
  //   Scalar  type for positions and vectors (will be cast as double)
  //   Index  type for indices (will be cast to int)
  // Input:
  //   mesh_file_name  path of .mesh file
  //   V  double matrix of vertex positions  #V by 3
  //   T  #T list of tet indices into vertex positions
  //   F  #F list of face indices into vertex positions
  //
  // Known bugs: Holes and regions are not supported
  //template <typename Scalar, typename Index>
  //IGL_INLINE bool writeHANDLE(
  //  const std::string mesh_file_name,
  //  const std::vector<std::vector<Scalar > > & V,
  //  const std::vector<std::vector<Index > > & T,
  //  const std::vector<std::vector<Index > > & F);

  // Templates:
  //   DerivedV  real-value: i.e. from MatrixXd
  //   DerivedT  integer-value: i.e. from MatrixXi
  //   DerivedF  integer-value: i.e. from MatrixXi
  // Input:
  //   mesh_file_name  path of .mesh file
  //   V  eigen double matrix #V by 3
  //   T  eigen int matrix #T by 4
  //   F  eigen int matrix #F by 3
  template <typename DerivedV, typename DerivedT, typename DerivedF, typename DerivedE, typename DerivedP>
  IGL_INLINE bool writeHANDLE(
    const std::string str,
    const Eigen::PlainObjectBase<DerivedV> & V, 
    const Eigen::PlainObjectBase<DerivedT> & T,
    const Eigen::PlainObjectBase<DerivedF> & F,
	const Eigen::PlainObjectBase<DerivedE> & E,
	const Eigen::PlainObjectBase<DerivedP> & P,
	const Eigen::PlainObjectBase<DerivedE> & BE,
	const Eigen::PlainObjectBase<DerivedF> & CF,
	const std::vector<std::vector<Eigen::VectorXi>>& G);

#ifndef IGL_STATIC_LIBRARY
#  include "writeHANDLE.cpp"
#endif

#endif /*IGL_ADDIN_WRITEHANDLE_H*/
