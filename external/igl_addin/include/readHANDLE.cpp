// This file is part of libigl, a simple c++ geometry processing library.
// 
// Copyright (C) 2013 Alec Jacobson <alecjacobson@gmail.com>
// 
// This Source Code Form is subject to the terms of the Mozilla Public License 
// v. 2.0. If a copy of the MPL was not distributed with this file, You can 
// obtain one at http://mozilla.org/MPL/2.0/.
#include "readHANDLE.h"

#include <cstdio>
#include <igl/verbose.h>

//template <typename Scalar, typename Index>
//IGL_INLINE bool readHANDLE(
//  const std::string mesh_file_name,
//  std::vector<std::vector<Scalar > > & V,
//  std::vector<std::vector<Index > > & T,
//  std::vector<std::vector<Index > > & F)
//{
//  using namespace std;
//  using namespace igl;
//  FILE * mesh_file = fopen(mesh_file_name.c_str(),"r");
//  if(NULL==mesh_file)
//  {
//    fprintf(stderr,"IOError: %s could not be opened...",mesh_file_name.c_str());
//    return false;
//  }
//#ifndef LINE_MAX
//#  define LINE_MAX 2048
//#endif
//  char line[LINE_MAX];
//  bool still_comments;
//  V.clear();
//  T.clear();
//  F.clear();
//
//  // eat comments at beginning of file
//  still_comments= true;
//  while(still_comments)
//  {
//    fgets(line,LINE_MAX,mesh_file);
//    still_comments = (line[0] == '#' || line[0] == '\n');
//  }
//
//  char str[LINE_MAX];
//  sscanf(line," %s",str);
//  // check that first word is MeshVersionFormatted
//  if(0!=strcmp(str,"MeshVersionFormatted"))
//  {
//    fprintf(stderr,
//      "Error: first word should be MeshVersionFormatted not %s\n",str);
//    fclose(mesh_file);
//    return false;
//  }
//  int one = -1;
//  if(2 != sscanf(line,"%s %d",str,&one))
//  {
//    // 1 appears on next line?
//    fscanf(mesh_file," %d",&one);
//  }
//  if(one != 1)
//  {
//    fprintf(stderr,"Error: second word should be 1 not %d\n",one);
//    fclose(mesh_file);
//    return false;
//  }
//
//  // eat comments
//  still_comments= true;
//  while(still_comments)
//  {
//    fgets(line,LINE_MAX,mesh_file);
//    still_comments = (line[0] == '#' || line[0] == '\n');
//  }
//
//  sscanf(line," %s",str);
//  // check that third word is Dimension
//  if(0!=strcmp(str,"Dimension"))
//  {
//    fprintf(stderr,"Error: third word should be Dimension not %s\n",str);
//    fclose(mesh_file);
//    return false;
//  }
//  int three = -1;
//  if(2 != sscanf(line,"%s %d",str,&three))
//  {
//    // 1 appears on next line?
//    fscanf(mesh_file," %d",&three);
//  }
//  if(three != 3)
//  {
//    fprintf(stderr,"Error: only Dimension 3 supported not %d\n",three);
//    fclose(mesh_file);
//    return false;
//  }
//
//  // eat comments
//  still_comments= true;
//  while(still_comments)
//  {
//    fgets(line,LINE_MAX,mesh_file);
//    still_comments = (line[0] == '#' || line[0] == '\n');
//  }
//
//  sscanf(line," %s",str);
//  // check that fifth word is Vertices
//  if(0!=strcmp(str,"Vertices"))
//  {
//    fprintf(stderr,"Error: fifth word should be Vertices not %s\n",str);
//    fclose(mesh_file);
//    return false;
//  }
//
//  //fgets(line,LINE_MAX,mesh_file);
//
//  int number_of_vertices;
//  if(1 != fscanf(mesh_file," %d",&number_of_vertices) || number_of_vertices > 1000000000)
//  {
//    fprintf(stderr,"Error: expecting number of vertices less than 10^9...\n");
//    fclose(mesh_file);
//    return false;
//  }
//  // allocate space for vertices
//  V.resize(number_of_vertices,vector<Scalar>(3,0));
//  int extra;
//  for(int i = 0;i<number_of_vertices;i++)
//  {
//    double x,y,z;
//    if(4 != fscanf(mesh_file," %lg %lg %lg %d",&x,&y,&z,&extra))
//    {
//      fprintf(stderr,"Error: expecting vertex position...\n");
//      fclose(mesh_file);
//      return false;
//    }
//    V[i][0] = x;
//    V[i][1] = y;
//    V[i][2] = z;
//  }
//
//  // eat comments
//  still_comments= true;
//  while(still_comments)
//  {
//    fgets(line,LINE_MAX,mesh_file);
//    still_comments = (line[0] == '#' || line[0] == '\n');
//  }
//
//  sscanf(line," %s",str);
//  // check that sixth word is Triangles
//  if(0!=strcmp(str,"Triangles"))
//  {
//    fprintf(stderr,"Error: sixth word should be Triangles not %s\n",str);
//    fclose(mesh_file);
//    return false;
//  }
//  int number_of_triangles;
//  if(1 != fscanf(mesh_file," %d",&number_of_triangles))
//  {
//    fprintf(stderr,"Error: expecting number of triangles...\n");
//    fclose(mesh_file);
//    return false;
//  }
//  // allocate space for triangles
//  F.resize(number_of_triangles,vector<Index>(3));
//  // triangle indices
//  int tri[3];
//  for(int i = 0;i<number_of_triangles;i++)
//  {
//    if(4 != fscanf(mesh_file," %d %d %d %d",&tri[0],&tri[1],&tri[2],&extra))
//    {
//      printf("Error: expecting triangle indices...\n");
//      return false;
//    }
//    for(int j = 0;j<3;j++)
//    {
//      F[i][j] = tri[j]-1;
//    }
//  }
//
//  // eat comments
//  still_comments= true;
//  while(still_comments)
//  {
//    fgets(line,LINE_MAX,mesh_file);
//    still_comments = (line[0] == '#' || line[0] == '\n');
//  }
//
//  sscanf(line," %s",str);
//  // check that sixth word is Triangles
//  if(0!=strcmp(str,"Tetrahedra"))
//  {
//    fprintf(stderr,"Error: seventh word should be Tetrahedra not %s\n",str);
//    fclose(mesh_file);
//    return false;
//  }
//  int number_of_tetrahedra;
//  if(1 != fscanf(mesh_file," %d",&number_of_tetrahedra))
//  {
//    fprintf(stderr,"Error: expecting number of tetrahedra...\n");
//    fclose(mesh_file);
//    return false;
//  }
//  // allocate space for tetrahedra
//  T.resize(number_of_tetrahedra,vector<Index>(4));
//  // tet indices
//  int a,b,c,d;
//  for(int i = 0;i<number_of_tetrahedra;i++)
//  {
//    if(5 != fscanf(mesh_file," %d %d %d %d %d",&a,&b,&c,&d,&extra))
//    {
//      fprintf(stderr,"Error: expecting tetrahedra indices...\n");
//      fclose(mesh_file);
//      return false;
//    }
//    T[i][0] = a-1;
//    T[i][1] = b-1;
//    T[i][2] = c-1;
//    T[i][3] = d-1;
//  }
//  fclose(mesh_file);
//  return true;
//}

#include <Eigen/Core>
#include <igl/list_to_matrix.h>

template <typename DerivedF>
IGL_INLINE bool read_variable(
	const std::string namestring,
	const int num_per_line,
	FILE * mesh_file,
	Eigen::PlainObjectBase<DerivedF>& F)
{
	 
	if(NULL==mesh_file)
	{
		fprintf(stderr,"IOError: %s is NULL...");
		return false;
	}

#ifndef LINE_MAX
#  define LINE_MAX 2048
#endif
	char line[LINE_MAX];
	bool still_comments;

	char str[LINE_MAX];

	// eat comments
	still_comments= true;
	while(still_comments)
	{
		fgets(line,LINE_MAX,mesh_file);
		still_comments = (line[0] == '#' || line[0] == '\n');
	}

	sscanf(line," %s",str);
	// check that sixth word is Triangles
	if(0!=strcmp(str,namestring.c_str()))
	{
		fprintf(stderr,"Error: should be name of the variable (%s) not %s\n",namestring,str);
		fclose(mesh_file);
		return false;
	}
	int number_of_variables;
	if(1 != fscanf(mesh_file," %d",&number_of_variables))
	{
		fprintf(stderr,"Error: expecting number of variables...\n");
		fclose(mesh_file);
		return false;
	}
	// allocate space for triangles
	F.resize(number_of_variables,num_per_line);
	// indices
	int * tri = new int[num_per_line];
	for(int i = 0;i<number_of_variables;i++)
	{
		switch(num_per_line)
		{
		case 1:
			if(num_per_line != fscanf(mesh_file," %d",&tri[0]))
			{
				printf("Error: expecting triangle indices...\n");
				return false;
			}
			break;
		case 2:
			if(num_per_line != fscanf(mesh_file," %d %d",&tri[0],&tri[1]))
			{
				printf("Error: expecting triangle indices...\n");
				return false;
			}
			break;
		case 3:
			if(num_per_line != fscanf(mesh_file," %d %d %d",&tri[0],&tri[1],&tri[2]))
			{
				printf("Error: expecting triangle indices...\n");
				return false;
			}
			break;
		case 4:
			if(num_per_line != fscanf(mesh_file," %d %d %d %d",&tri[0],&tri[1],&tri[2],&tri[3]))
			{
				printf("Error: expecting triangle indices...\n");
				return false;
			}
			break;
		case 5:
			if(num_per_line != fscanf(mesh_file," %d %d %d %d %d",&tri[0],&tri[1],&tri[2],&tri[3],&tri[4]))
			{
				printf("Error: expecting triangle indices...\n");
				return false;
			}
			break;
		default:
			printf("Error: unsupported num_per_line(%d) ...\n",num_per_line);
			return false;
		}

		for(int j = 0; j<num_per_line; j++)
		{
			F(i,j) = tri[j]-1;
		}
	}
	delete [] tri;
}

IGL_INLINE bool read_group(
	const std::string namestring,
	FILE * mesh_file,
	std::vector<Eigen::VectorXi> & G
	)
{

	if(NULL==mesh_file)
	{
		fprintf(stderr,"IOError: %s is NULL...");
		return false;
	}

#ifndef LINE_MAX
#  define LINE_MAX 2048
#endif
	char line[LINE_MAX];
	bool still_comments;

	char str[LINE_MAX];

	// eat comments
	still_comments= true;
	while(still_comments)
	{
		fgets(line,LINE_MAX,mesh_file);
		still_comments = (line[0] == '#' || line[0] == '\n');
	}

	sscanf(line," %s",str);
	// check that sixth word is Triangles
	if(0!=strcmp(str,namestring.c_str()))
	{
		fprintf(stderr,"Error: should be name of the variable (%s) not %s\n",namestring,str);
		fclose(mesh_file);
		return false;
	}
	int number_of_variables;
	if(1 != fscanf(mesh_file," %d\n",&number_of_variables))// wangyu change this line to %d\n
	{
		fprintf(stderr,"Error: expecting number of variables...\n");
		fclose(mesh_file);
		return false;
	}
	// allocate space for triangles
	//G.resize(number_of_variables);
	// indices
#ifndef GROUP_LINE_MAX
	#define GROUP_LINE_MAX 81920
#endif
	char lineBuf[GROUP_LINE_MAX];
	char tempBuf[GROUP_LINE_MAX];
	int number[GROUP_LINE_MAX];
	//std::string groupline;
	char * tok;
	for(int i = 0;i<number_of_variables;i++)
	{
		// http://stackoverflow.com/questions/19280402/read-unknown-number-of-integers-in-n-lines
		fgets(lineBuf,GROUP_LINE_MAX,mesh_file);
		//std::getline(mesh_file, groupline);
		int num = 0;
		tok = strtok(lineBuf, " \n");
		while(tok)
		{
			strcpy(tempBuf, tok);
			number[num++] = atoi(tempBuf);
			tok = strtok(NULL, " \n");
		}
		Eigen::VectorXi newline(num);
		for (int j=0; j<num; j++)
		{
			newline(j) = number[j]-1;
		}
		G.push_back(newline);
	}
}

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
	std::vector<std::vector<Eigen::VectorXi>>& G)
{
  using namespace std;
  using namespace igl;
  FILE * mesh_file = fopen(mesh_file_name.c_str(),"r");
  if(NULL==mesh_file)
  {
    fprintf(stderr,"IOError: %s could not be opened...",mesh_file_name.c_str());
    return false;
  }
#ifndef LINE_MAX
#  define LINE_MAX 2048
#endif
  char line[LINE_MAX];
  bool still_comments;

  // eat comments at beginning of file
  still_comments= true;
  while(still_comments)
  {
    fgets(line,LINE_MAX,mesh_file);
    still_comments = (line[0] == '#' || line[0] == '\n');
  }

  char str[LINE_MAX];
  sscanf(line," %s",str);
  // check that first word is MeshVersionFormatted
  if(0!=strcmp(str,"MeshVersionFormatted"))
  {
    fprintf(stderr,
      "Error: first word should be MeshVersionFormatted not %s\n",str);
    fclose(mesh_file);
    return false;
  }
  int one = -1;
  if(2 != sscanf(line,"%s %d",str,&one))
  {
    // 1 appears on next line?
    fscanf(mesh_file," %d",&one);
  }
  if(one != 1)
  {
    fprintf(stderr,"Error: second word should be 1 not %d\n",one);
    fclose(mesh_file);
    return false;
  }

  // eat comments
  still_comments= true;
  while(still_comments)
  {
    fgets(line,LINE_MAX,mesh_file);
    still_comments = (line[0] == '#' || line[0] == '\n');
  }

  sscanf(line," %s",str);
  // check that third word is Dimension
  if(0!=strcmp(str,"Dimension"))
  {
    fprintf(stderr,"Error: third word should be Dimension not %s\n",str);
    fclose(mesh_file);
    return false;
  }
  int three = -1;
  if(2 != sscanf(line,"%s %d",str,&three))
  {
    // 1 appears on next line?
    fscanf(mesh_file," %d",&three);
  }
  if(three != 3)
  {
    fprintf(stderr,"Error: only Dimension 3 supported not %d\n",three);
    fclose(mesh_file);
    return false;
  }

  // eat comments
  still_comments= true;
  while(still_comments)
  {
    fgets(line,LINE_MAX,mesh_file);
    still_comments = (line[0] == '#' || line[0] == '\n');
  }

  sscanf(line," %s",str);
  // check that fifth word is Vertices
  if(0!=strcmp(str,"Vertices"))
  {
    fprintf(stderr,"Error: fifth word should be Vertices not %s\n",str);
    fclose(mesh_file);
    return false;
  }

  //fgets(line,LINE_MAX,mesh_file);

  int number_of_vertices;
  if(1 != fscanf(mesh_file," %d",&number_of_vertices) || number_of_vertices > 1000000000)
  {
    fprintf(stderr,"Error: expecting number of vertices less than 10^9...\n");
    fclose(mesh_file);
    return false;
  }
  // allocate space for vertices
  V.resize(number_of_vertices,3);
  int extra;
  for(int i = 0;i<number_of_vertices;i++)
  {
    double x,y,z;
    if(4 != fscanf(mesh_file," %lg %lg %lg %d",&x,&y,&z,&extra))
    {
      fprintf(stderr,"Error: expecting vertex position...\n");
      fclose(mesh_file);
      return false;
    }
    V(i,0) = x;
    V(i,1) = y;
    V(i,2) = z;
  }

  Eigen::MatrixXi FF = Eigen::MatrixXi(0,4);
  read_variable( "Triangles", 4, mesh_file, FF);
  F = FF.leftCols<3>(); // It seems that it will give incorrect result if using F = F.leftCols<3>();
  
  Eigen::MatrixXi TT = Eigen::MatrixXi(0,5);
  read_variable( "Tetrahedra", 5, mesh_file, TT);
  T = TT.leftCols<4>();

  Eigen::MatrixXi EE = Eigen::MatrixXi(0,3);
  read_variable( "Edges", 3, mesh_file, EE);
  E = EE.leftCols<2>();

  read_variable( "Points", 1, mesh_file, P);
  read_variable( "BoneEdges", 2, mesh_file, BE);
  read_variable( "CageFaces", 3, mesh_file, CF);

  // eat comments
  still_comments= true;
  while(still_comments)
  {
	  fgets(line,LINE_MAX,mesh_file);
	  still_comments = (line[0] == '#' || line[0] == '\n');
  }

  sscanf(line," %s",str);
  if(0!=strcmp(str,"Groups"))
  {
	  fprintf(stderr,"Error: should be name of the variable (%s) not %s\n","Groups",str);
	  fclose(mesh_file);
	  return false;
  }

  std::vector<Eigen::VectorXi> GP;
  read_group("GroupPoints",mesh_file,GP);

  std::vector<Eigen::VectorXi> GBE;
  read_group("GroupBoneEdges",mesh_file,GBE);

  std::vector<Eigen::VectorXi> GCF;
  read_group("GroupCageFaces",mesh_file,GCF);

  G.clear();
  G.push_back(GP);
  G.push_back(GBE);
  G.push_back(GCF);

  printf("Read handle successful: V(%d),T(%d),F(%d),E(%d),P(%d),BE(%d),CF(%d)\n", 
	  V.rows(), T.rows(), F.rows(), E.rows(), P.rows(), BE.rows(), CF.rows());
  printf("Group:\n");
  printf("\tGroup Points:\n");
  for (int i=0; i<GP.size(); i++)
  {
	  printf("\t\t%d:",i);
	  for (int j=0; j<GP[i].rows(); j++)
		printf(" %d ",GP[i](j,0));
	  printf("\n");
  }
  printf("\tGroup BoneEdges:\n");
  for (int i=0; i<GBE.size(); i++)
  {
	  printf("\t\t%d:",i);
	  for (int j=0; j<GBE[i].rows(); j++)
		  printf(" %d ",GBE[i](j,0));
	  printf("\n");
  }  
  printf("\tGroup CageFaces:\n");
  for (int i=0; i<GCF.size(); i++)
  {
	  printf("\t\t%d:",i);
	  for (int j=0; j<GCF[i].rows(); j++)
		  printf(" %d ",GCF[i](j,0));
	  printf("\n");
  }  


  fclose(mesh_file);
  return true;
}
//{
//  std::vector<std::vector<double> > vV,vT,vF;
//  bool success = igl::readMESH(mesh_file_name,vV,vT,vF);
//  if(!success)
//  {
//    // readMESH already printed error message to std err
//    return false;
//  }
//  bool V_rect = igl::list_to_matrix(vV,V);
//  if(!V_rect)
//  {
//    // igl::list_to_matrix(vV,V) already printed error message to std err
//    return false;
//  }
//  bool T_rect = igl::list_to_matrix(vT,T);
//  if(!T_rect)
//  {
//    // igl::list_to_matrix(vT,T) already printed error message to std err
//    return false;
//  }
//  bool F_rect = igl::list_to_matrix(vF,F);
//  if(!F_rect)
//  {
//    // igl::list_to_matrix(vF,F) already printed error message to std err
//    return false;
//  }
//  assert(V.cols() == 3);
//  assert(T.cols() == 4);
//  assert(F.cols() == 3);
//  return true;
//}

#ifdef IGL_STATIC_LIBRARY
// Explicit template specialization
// generated by autoexplicit.sh MODIFIED BY WANGYU
template bool readHANDLE<
	Eigen::Matrix<double, -1, 3, 0, -1, 3>, 
	Eigen::Matrix<int, -1, 3, 0, -1, 3>, 
	Eigen::Matrix<int, -1, -1, 0, -1, -1>, 
	Eigen::Matrix<int, -1, -1, 0, -1, -1>, 
	Eigen::Matrix<int, -1, -1, 0, -1, -1>  
>(// WANGYU not sure about this
	std::basic_string<char, std::char_traits<char>, std::allocator<char> >, 
	Eigen::PlainObjectBase<Eigen::Matrix<double, -1, 3, 0, -1, 3> >&, 
	Eigen::PlainObjectBase<Eigen::Matrix<int, -1, -1, 0, -1, -1> >&, 
	Eigen::PlainObjectBase<Eigen::Matrix<int, -1, 3, 0, -1, 3> >&,
	Eigen::PlainObjectBase<Eigen::Matrix<int, -1, -1, 0, -1, -1> >&, 
	Eigen::PlainObjectBase<Eigen::Matrix<int, -1, -1, 0, -1, -1> >&);

// generated by autoexplicit.sh MODIFIED BY WANGYU
template bool readHANDLE<
	Eigen::Matrix<double, -1, -1, 0, -1, -1>, 
	Eigen::Matrix<int, -1, -1, 0, -1, -1>, 
	Eigen::Matrix<int, -1, -1, 0, -1, -1>, 
	Eigen::Matrix<int, -1, -1, 0, -1, -1>, 
	Eigen::Matrix<int, -1, -1, 0, -1, -1>  
>(
	std::basic_string<char, std::char_traits<char>, std::allocator<char> >, 
	Eigen::PlainObjectBase<Eigen::Matrix<double, -1, -1, 0, -1, -1> >&, 
	Eigen::PlainObjectBase<Eigen::Matrix<int, -1, -1, 0, -1, -1> >&, 
	Eigen::PlainObjectBase<Eigen::Matrix<int, -1, -1, 0, -1, -1> >&,
	Eigen::PlainObjectBase<Eigen::Matrix<int, -1, -1, 0, -1, -1> >&, 
	Eigen::PlainObjectBase<Eigen::Matrix<int, -1, -1, 0, -1, -1> >&);

template bool readHANDLE<class Eigen::Matrix<double, -1, -1, 0, -1, -1>, class Eigen::Matrix<int, -1, -1, 0, -1, -1>, class Eigen::Matrix<int, -1, -1, 0, -1, -1>, class Eigen::Matrix<int, -1, -1, 0, -1, -1>, class Eigen::Matrix<int, -1, -1, 0, -1, -1> >(class std::basic_string<char, struct std::char_traits<char>, class std::allocator<char> >, class Eigen::PlainObjectBase<class Eigen::Matrix<double, -1, -1, 0, -1, -1> > &, class Eigen::PlainObjectBase<class Eigen::Matrix<int, -1, -1, 0, -1, -1> > &, class Eigen::PlainObjectBase<class Eigen::Matrix<int, -1, -1, 0, -1, -1> > &, class Eigen::PlainObjectBase<class Eigen::Matrix<int, -1, -1, 0, -1, -1> > &, class Eigen::PlainObjectBase<class Eigen::Matrix<int, -1, -1, 0, -1, -1> > &, class Eigen::PlainObjectBase<class Eigen::Matrix<int, -1, -1, 0, -1, -1> > &, class Eigen::PlainObjectBase<class Eigen::Matrix<int, -1, -1, 0, -1, -1> > &, class std::vector<class std::vector<class Eigen::Matrix<int, -1, 1, 0, -1, 1>, class std::allocator<class Eigen::Matrix<int, -1, 1, 0, -1, 1> > >, class std::allocator<class std::vector<class Eigen::Matrix<int, -1, 1, 0, -1, 1>, class std::allocator<class Eigen::Matrix<int, -1, 1, 0, -1, 1> > > > > &);
#endif
