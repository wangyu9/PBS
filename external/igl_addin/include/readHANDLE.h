// This file is part of libigl, a simple c++ geometry processing library.
// 
// Copyright (C) 2013 Alec Jacobson <alecjacobson@gmail.com>
// 
// This Source Code Form is subject to the terms of the Mozilla Public License 
// v. 2.0. If a copy of the MPL was not distributed with this file, You can 
// obtain one at http://mozilla.org/MPL/2.0/.
#ifndef IGL_ADDIN_READHANDLE_H
#define IGL_ADDIN_READHANDLE_H
#include <igl/igl_inline.h>

#include <string>
#include <vector>
#include <Eigen/Core>


  // load a tetrahedral volume mesh from a .mesh file
  //
  // Templates:
  //   Scalar  type for positions and vectors (will be read as double and cast
  //     to Scalar)
  //   Index  type for indices (will be read as int and cast to Index)
  // Input:
  //   mesh_file_name  path of .mesh file
  // Outputs:
  //   V  double matrix of vertex positions  #V by 3
  //   T  #T list of tet indices into vertex positions
  //   F  #F list of face indices into vertex positions
  //
  // Known bugs: Holes and regions are not supported
  //template <typename Scalar, typename Index>
  //IGL_INLINE bool readHANDLE(
  //  const std::string mesh_file_name,
  //  std::vector<std::vector<Scalar > > & V,
  //  std::vector<std::vector<Index > > & T,
  //  std::vector<std::vector<Index > > & F);

  // Input:
  //   mesh_file_name  path of .mesh file
  // Outputs:
  //   V  eigen double matrix #V by 3
  //   T  eigen int matrix #T by 4
  //   F  eigen int matrix #F by 3
  template <typename DerivedV, typename DerivedT, typename DerivedF, typename DerivedE, typename DerivedP>
  IGL_INLINE bool readHANDLE(
	  const std::string mesh_file_name,
	  Eigen::PlainObjectBase<DerivedV>& V,
	  Eigen::PlainObjectBase<DerivedT>& T,
	  Eigen::PlainObjectBase<DerivedF>& F,
	  Eigen::PlainObjectBase<DerivedE>& E,
	  Eigen::PlainObjectBase<DerivedP>& P,
	  Eigen::PlainObjectBase<DerivedE>& BE,
	  Eigen::PlainObjectBase<DerivedF>& CF,
	  std::vector<std::vector<Eigen::VectorXi>>& G);


#ifndef IGL_STATIC_LIBRARY
#  include "readHANDLE.cpp"
#endif

#endif /*IGL_ADDIN_READHANDLE_H*/
