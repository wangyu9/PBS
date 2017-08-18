// IGL Viewer - Copyright (c) 2013 ETH Zurich. All rights reserved.
// Helper functions to load a texture from a TGA file and convert it to a format compatible with OpenGL

#include <iostream>

#include "texture_png.h"

#include "types.h"

bool max_cols(
              const vector<Vector > &A,
              Vector & max_A)
{
  if(A.size() == 0)
  {
    printf("ERROR: no max min computed. Given list was empty.\n");
    return false;
  }
  // size of list elements (must all be the same)
  size_t len = A[0].size();
  max_A.resize(len,-1e16);
  // loop over lists in A
  for(size_t i = 0; i < A.size(); i++)
  {
    if(A[i].size() != len)
    {
      printf("ERROR: A[%lu].size() = %lu != A[0].size() = %lu\n",
             i,
             A[i].size(),
             len);
      return false;
    }
    for(size_t j = 0; j < len; j++)
    {
      max_A[j] = max(max_A[j],A[i][j]);
    }
  }
  return true;
}

bool min_cols(
              const vector<Vector > &A,
              Vector & min_A)
{
  if(A.size() == 0)
  {
    printf("ERROR: no min min computed. Given list was empty.\n");
    return false;
  }
  // size of list elements (must all be the same)
  size_t len = A[0].size();
  min_A.resize(len,1e16);
  // loop over lists in A
  for(size_t i = 0; i < A.size(); i++)
  {
    if(A[i].size() != len)
    {
      printf("ERROR: A[%lu].size() = %lu != A[0].size() = %lu\n",
             i,
             A[i].size(),
             len);
      return false;
    }
    for(size_t j = 0; j < len; j++)
    {
      min_A[j] = min(min_A[j],A[i][j]);
    }
  }
  return true;
}

GLuint texture_from_png(const string tga_file)
{
  //// read pixels to tga file
  //FILE * imgFile;
  //// "-" as input file name is code for read from stdin
  //imgFile = fopen(tga_file.c_str(),"r");
  //if(NULL==imgFile)
  //{
  //  printf("IOError: %s could not be opened...",tga_file.c_str());
  //  return false;
  //}
  //// gliReadTGA annoyingly uses char * instead of const char *
  //size_t len = tga_file.length();
  //char* tga_file_char = new char [ len + 1 ];
  //strcpy( tga_file_char, tga_file.c_str() );
  //// read image
  //gliGenericImage* img = gliReadTGA(imgFile, tga_file_char, 0, 0);
  //// clean up filename buffer
  //delete[] tga_file_char;
  //fclose( imgFile );
  
	YImage* img = read_png(tga_file);

  //// Warn if input .tga width or height is not divisble by 4
  //if( img->width%4 != 0)
  //{
  //  fprintf(stderr,
  //    "WARNING: width of %s (= %d) not divisible by 4."
  //    "Output will likely be incorrect.\n",
  //    tga_file.c_str(),
  //    img->width);
  //}
  //if( img->height%4 != 0)
  //{
  //  fprintf(stderr,
  //    "WARNING: height of %s (= %d) not divisible by 4."
  //    "Output will likely be incorrect.\n",
  //    tga_file.c_str(),
  //    img->height);
  //}
  
  // set up texture mapping parameters and generate texture id
  GLuint texture_id;
  glGenTextures(1,&texture_id);
  glBindTexture(GL_TEXTURE_2D, texture_id);
  // Texture parameters
  float empty[] = {1.0f,1.0f,1.0f,0.0f};
  glTexParameterfv(GL_TEXTURE_2D,GL_TEXTURE_BORDER_COLOR,empty);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
  //glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER,
  //  GL_LINEAR_MIPMAP_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
                  GL_LINEAR);
  
  // OpenGL by default tries to read data in multiples of 4, if our data is
  // only RGB or BGR and the width is not divible by 4 then we need to alert
  // opengl
  //if((img->width % 4) != 0 && 
  //   (img->format == GL_RGB || 
  //    img->format == GL_BGR))
  if ((img->width()%4)!=0)
  {
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
  }
  
  // Load texture
  //glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, img->width,
	 // img->height, 0, img->format, GL_UNSIGNED_BYTE,
	 // img->pixels);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, img->width(),
               img->height(), 0, GL_RGBA, GL_UNSIGNED_BYTE,
               img->data());
  //wangyu img->save("test_png_load.png");
#ifdef __DEBUG__
  printf("%s (%d,%d) --> %d\n",
         tga_file.c_str(),
         img->width,
         img->height,
         texture_id);
#endif
  return texture_id;
}

YImage* read_png(const std::string tga_file)
{
	YImage* img = new YImage();
	img->load(tga_file.c_str());
	return img;
  //// read pixels to tga file
  //FILE * imgFile;
  //// "-" as input file name is code for read from stdin
  //imgFile = fopen(tga_file.c_str(),"r");
  //if(NULL==imgFile)
  //{
  //  printf("IOError: %s could not be opened...",tga_file.c_str());
  //  return false;
  //}
  //// gliReadTGA annoyingly uses char * instead of const char *
  //size_t len = tga_file.length();
  //char* tga_file_char = new char [ len + 1 ];
  //strcpy( tga_file_char, tga_file.c_str() );
  //// read image
  //gliGenericImage* img = gliReadTGA(imgFile, tga_file_char, 0, 0);
  //// clean up filename buffer
  //delete[] tga_file_char;
  //fclose( imgFile );
  //return img;
}

bool flipCoord(
               const Eigen::MatrixXd *A,
               const int k,
				Eigen::MatrixXd *B)
{
  // resize B to match A
  *B = *A;
  
  if(2<=k)
  {
    printf("ERROR: A->cols() <= %lu\n",A->cols());
    return false;
  }
  
  ScalarType max_Ak = A->col(k).maxCoeff();
  ScalarType min_Ak = A->col(k).minCoeff();

  B->col(k) = -1.*A->col(k) +(max_Ak + min_Ak)*Eigen::Matrix<ScalarType,Eigen::Dynamic,1>::Ones(B->rows(),1);
  return true;
}


bool copyXY(
            const Eigen::MatrixXd *A,
			Eigen::MatrixXd *B)
{
  if(A->cols()<2)
  {
    printf("ERROR: A->cols() < 2\n");
    return false;
  }

  // resize B to match A
  *B = A->block(0, 0, A->rows(), 2);
  return true;
}


void normalize(
               const Eigen::MatrixXd *A,
	Eigen::MatrixXd *B)
{
  Eigen::Matrix<ScalarType, 2, 1>  max_A = A->colwise().maxCoeff();
  Eigen::Matrix<ScalarType, 2, 1>  min_A = A->colwise().minCoeff();
  
  
  // make room in output
  B->resize(A->rows(),2);
  
  B->col(0) = (A->col(0).array() - max_A[0]) / (max_A[0] -min_A[0]);
  B->col(1) = (A->col(1).array() - max_A[1]) / (max_A[1] -min_A[1]);
}

// compute max value in A
double maxv(const Vector &A)
{
  double max_A = -1e16;
  for(size_t i = 0; i<A.size(); i++)
  {
    max_A = max(max_A,A[i]);
  }
  return max_A;
}

// compute min value in A
double minv(const Vector &A)
{
  double min_A = 1e16;
  for(size_t i = 0; i<A.size(); i++)
  {
    min_A = min(min_A,A[i]);
  }
  return min_A;
}

bool normalized(
                const Eigen::MatrixXd *A)
{
  return A->maxCoeff() <= 1 && A->minCoeff() >= 0;
}
