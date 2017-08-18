// This file is part of libigl, a simple c++ geometry processing library.
// 
// Copyright (C) 2013 Alec Jacobson <alecjacobson@gmail.com>
// 
// This Source Code Form is subject to the terms of the Mozilla Public License 
// v. 2.0. If a copy of the MPL was not distributed with this file, You can 
// obtain one at http://mozilla.org/MPL/2.0/.
#include "writeHANDLE.h"

#include <igl/verbose.h>
#include <igl/matrix_to_list.h>
#include <Eigen/Core>

#include <iostream>
#include <fstream>
#include <cstdio>

//template <typename Scalar, typename Index>
//IGL_INLINE bool writeHANDLE(
//  const std::string mesh_file_name,
//  const std::vector<std::vector<Scalar > > & V,
//  const std::vector<std::vector<Index > > & T,
//  const std::vector<std::vector<Index > > & F)
//{
//  Eigen::MatrixXd mV;
//  Eigen::MatrixXi mT,mF;
//  bool is_rect;
//  is_rect = list_to_matrix(V,mV);
//  if(!is_rect)
//  {
//    return false;
//  }
//  is_rect = list_to_matrix(T,mT);
//  if(!is_rect)
//  {
//    return false;
//  }
//  is_rect = list_to_matrix(F,mF);
//  if(!is_rect)
//  {
//    return false;
//  }
//  return igl::writeMESH(mesh_file_name,mV,mT,mF);
//}





template <typename DerivedF>
IGL_INLINE bool write_variable(
	const std::string namestring,
	const int num_per_line,
	FILE * mesh_file,
	const Eigen::PlainObjectBase<DerivedF>& F)
{
	// print namestring
	fprintf(mesh_file,namestring.c_str());
	fprintf(mesh_file,"\n");
	// print number of variables
	int number_of_variables = F.rows();
	fprintf(mesh_file,"%d\n",number_of_variables);
	// loop over variables
	for(int i=0; i<number_of_variables; i++)
	{
		
		switch(num_per_line)
		{
		case 1:
			fprintf(mesh_file,"%d\n", 
				(int)F(i,0)+1);
			break;
		case 2:
			fprintf(mesh_file,"%d %d\n", 
				(int)F(i,0)+1, 
				(int)F(i,1)+1);
			break;
		case 3:
			fprintf(mesh_file,"%d %d %d\n", 
				(int)F(i,0)+1, 
				(int)F(i,1)+1, 
				(int)F(i,2)+1);
			break;
		case 4:
			fprintf(mesh_file,"%d %d %d %d\n", 
				(int)F(i,0)+1, 
				(int)F(i,1)+1, 
				(int)F(i,2)+1, 
				(int)F(i,3)+1);
			break;
		case 5:
			fprintf(mesh_file,"%d %d %d %d %d\n", 
				(int)F(i,0)+1, 
				(int)F(i,1)+1, 
				(int)F(i,2)+1, 
				(int)F(i,3)+1, 
				(int)F(i,4)+1);
			break;
		default:
			printf("Error: unsupported num_per_line(%d) ...\n",num_per_line);
			return false;
		}
	}
	return true;
}


IGL_INLINE bool write_group(
	const std::string namestring,
	FILE * mesh_file,
	const std::vector<Eigen::VectorXi> & G
	)
{
	fprintf(mesh_file,namestring.c_str());
	fprintf(mesh_file,"\n");
	fprintf(mesh_file,"%d\n",G.size());
	for (int i=0; i<G.size(); i++)
	{
		const Eigen::VectorXi& I = G[i];
		for (int j=0; j<I.rows(); j++)
		{
			fprintf(mesh_file,"%d ",I(j)+1);
		}
		fprintf(mesh_file,"\n");
	}
	return true;
}


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
	const std::vector<std::vector<Eigen::VectorXi>>& G)
{
  using namespace std;
  using namespace igl;
  using namespace Eigen;

  //// This is (surprisingly) slower than the C-ish code below
  //ofstream mesh_file;
  //mesh_file.open(str.c_str());
  //if(!mesh_file.is_open())
  //{
  //  cerr<<"IOError: "<<str<<" could not be opened..."<<endl;
  //  return false;
  //}
  //IOFormat format(FullPrecision,DontAlignCols," ","\n",""," 1","","");
  //mesh_file<<"MeshVersionFormatted 1\n";
  //mesh_file<<"Dimension 3\n";
  //mesh_file<<"Vertices\n";
  //mesh_file<<V.rows()<<"\n";
  //mesh_file<<V.format(format)<<"\n";
  //mesh_file<<"Triangles\n";
  //mesh_file<<F.rows()<<"\n";
  //mesh_file<<(F.array()+1).eval().format(format)<<"\n";
  //mesh_file<<"Tetrahedra\n";
  //mesh_file<<T.rows()<<"\n";
  //mesh_file<<(T.array()+1).eval().format(format)<<"\n";
  //mesh_file.close();

  FILE * mesh_file = fopen(str.c_str(),"w");
  if(NULL==mesh_file)
  {
    fprintf(stderr,"IOError: %s could not be opened...",str.c_str());
    return false;
  }
  // print header
  fprintf(mesh_file,"MeshVersionFormatted 1\n");
  fprintf(mesh_file,"Dimension 3\n");
  // print tet vertices
  fprintf(mesh_file,"Vertices\n");
  // print number of tet vertices
  int number_of_tet_vertices = V.rows();
  fprintf(mesh_file,"%d\n",number_of_tet_vertices);
  // loop over tet vertices
  for(int i = 0;i<number_of_tet_vertices;i++)
  {
    // print position of ith tet vertex
    fprintf(mesh_file,"%.17lg %.17lg %.17lg 1\n",
      (double)V(i,0),
      (double)V(i,1),
      (double)V(i,2));
  }
  verbose("WARNING: save_mesh() assumes that vertices have"
      " same indices in surface as volume...\n");

  //// print faces
  //fprintf(mesh_file,"Triangles\n");
  //// print number of triangles
  //int number_of_triangles = F.rows();
  //fprintf(mesh_file,"%d\n",number_of_triangles);
  //// loop over faces
  //for(int i = 0;i<number_of_triangles;i++)
  //{
  //  // loop over vertices in face
  //  fprintf(mesh_file,"%d %d %d 1\n", 
  //    (int)F(i,0)+1, 
  //    (int)F(i,1)+1, 
  //    (int)F(i,2)+1);
  //}

  Eigen::MatrixXi FF = Eigen::MatrixXi::Zero(F.rows(),4);
  FF.block(0,0,FF.rows(),3) = F;
  // Note: implicitly set the last col to be zeros
  write_variable("Triangles",4,mesh_file,FF);

  Eigen::MatrixXi TT = Eigen::MatrixXi::Zero(T.rows(),5);
  TT.block(0,0,TT.rows(),4) = T;
  // Note: implicitly set the last col to be zeros
  write_variable("Tetrahedra",5,mesh_file,TT);

  Eigen::MatrixXi EE = Eigen::MatrixXi::Zero(E.rows(),3);
  EE.block(0,0,EE.rows(),2) = E;
  // Note: implicitly set the last col to be zeros
  write_variable("Edges",3,mesh_file,EE);

  write_variable("Points",1,mesh_file,P);

  write_variable("BoneEdges",2,mesh_file,BE);
  write_variable("CageFaces",3,mesh_file,CF);

  fprintf(mesh_file,"Groups\n");

  write_group("GroupPoints",mesh_file,G[0]);
  write_group("GroupBoneEdges",mesh_file,G[1]);
  write_group("GroupCageFaces",mesh_file,G[2]);

  //// print tetrahedra
  //fprintf(mesh_file,"Tetrahedra\n");
  //int number_of_tetrahedra = T.rows();
  //// print number of tetrahedra
  //fprintf(mesh_file,"%d\n",number_of_tetrahedra);
  //// loop over tetrahedra
  //for(int i = 0; i < number_of_tetrahedra;i++)
  //{
  //  // mesh standard uses 1-based indexing
  //  fprintf(mesh_file, "%d %d %d %d 1\n",
  //    (int)T(i,0)+1,
  //    (int)T(i,1)+1,
  //    (int)T(i,2)+1,
  //    (int)T(i,3)+1);
  //}


  fclose(mesh_file);
  return true;
}

#ifdef IGL_STATIC_LIBRARY
// Explicit template specialization
//template bool igl::writeMESH<Eigen::Matrix<double, -1, -1, 0, -1, -1>, Eigen::Matrix<int, -1, -1, 0, -1, -1>, Eigen::Matrix<int, -1, -1, 0, -1, -1> >(std::basic_string<char, std::char_traits<char>, std::allocator<char> >, Eigen::MatrixBase<Eigen::Matrix<double, -1, -1, 0, -1, -1> > const&, Eigen::MatrixBase<Eigen::Matrix<int, -1, -1, 0, -1, -1> > const&, Eigen::MatrixBase<Eigen::Matrix<int, -1, -1, 0, -1, -1> > const&);
//template bool writeHANDLE<
//	Eigen::Matrix<double, -1, -1, 0, -1, -1>, 
//	Eigen::Matrix<int, -1, -1, 0, -1, -1>, 
//	Eigen::Matrix<int, -1, -1, 0, -1, -1> >
//	(
//	std::basic_string<char, std::char_traits<char>, std::allocator<char> >, 
//	Eigen::PlainObjectBase<Eigen::Matrix<double, -1, -1, 0, -1, -1> > const&, 
//	Eigen::PlainObjectBase<Eigen::Matrix<int, -1, -1, 0, -1, -1> > const&, 
//	Eigen::PlainObjectBase<Eigen::Matrix<int, -1, -1, 0, -1, -1> > const&);

//copied from vs by wangyu//template bool writeHANDLE<class Eigen::Matrix<double,-1,-1,0,-1,-1>,class Eigen::Matrix<int,-1,-1,0,-1,-1>,class Eigen::Matrix<int,-1,-1,0,-1,-1>,class Eigen::Matrix<int,-1,-1,0,-1,-1>,class Eigen::Matrix<int,-1,-1,0,-1,-1> >(class std::basic_string<char,struct std::char_traits<char>,class std::allocator<char> >,class Eigen::PlainObjectBase<class Eigen::Matrix<double,-1,-1,0,-1,-1> > const &,class Eigen::PlainObjectBase<class Eigen::Matrix<int,-1,-1,0,-1,-1> > const &,class Eigen::PlainObjectBase<class Eigen::Matrix<int,-1,-1,0,-1,-1> > const &,class Eigen::PlainObjectBase<class Eigen::Matrix<int,-1,-1,0,-1,-1> > const &,class Eigen::PlainObjectBase<class Eigen::Matrix<int,-1,-1,0,-1,-1> > const &,class Eigen::PlainObjectBase<class Eigen::Matrix<int,-1,-1,0,-1,-1> > const &,class Eigen::PlainObjectBase<class Eigen::Matrix<int,-1,-1,0,-1,-1> > const &,class std::vector<class std::vector<class Eigen::Matrix<int,-1,1,0,-1,1>,class std::allocator<class Eigen::Matrix<int,-1,1,0,-1,1> > >,class std::allocator<class std::vector<class Eigen::Matrix<int,-1,1,0,-1,1>,class std::allocator<class Eigen::Matrix<int,-1,1,0,-1,1> > > > > const &);
template bool writeHANDLE<class Eigen::Matrix<double,-1,-1,0,-1,-1>,class Eigen::Matrix<int,-1,-1,0,-1,-1>,class Eigen::Matrix<int,-1,-1,0,-1,-1>,class Eigen::Matrix<int,-1,-1,0,-1,-1>,class Eigen::Matrix<int,-1,-1,0,-1,-1> >(class std::basic_string<char,struct std::char_traits<char>,class std::allocator<char> >,class Eigen::PlainObjectBase<class Eigen::Matrix<double,-1,-1,0,-1,-1> > const &,class Eigen::PlainObjectBase<class Eigen::Matrix<int,-1,-1,0,-1,-1> > const &,class Eigen::PlainObjectBase<class Eigen::Matrix<int,-1,-1,0,-1,-1> > const &,class Eigen::PlainObjectBase<class Eigen::Matrix<int,-1,-1,0,-1,-1> > const &,class Eigen::PlainObjectBase<class Eigen::Matrix<int,-1,-1,0,-1,-1> > const &,class Eigen::PlainObjectBase<class Eigen::Matrix<int,-1,-1,0,-1,-1> > const &,class Eigen::PlainObjectBase<class Eigen::Matrix<int,-1,-1,0,-1,-1> > const &,class std::vector<class std::vector<class Eigen::Matrix<int,-1,1,0,-1,1>,class std::allocator<class Eigen::Matrix<int,-1,1,0,-1,1> > >,class std::allocator<class std::vector<class Eigen::Matrix<int,-1,1,0,-1,1>,class std::allocator<class Eigen::Matrix<int,-1,1,0,-1,1> > > > > const &);
//copied from vs by wangyu

#endif
