//
//  Deform.h
//  Preview3D
//
//  Created by Olga Diamanti on 6/10/11.
//  Copyright 2011 __MyCompanyName__. All rights reserved.
//

#ifndef Preview3D_Deform_h
#define Preview3D_Deform_h
#undef max
#undef min
#include "cotmatrix.h"
#include "cotangent.h"
#include "adjacency_list.h"

#include "vf.h"
#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET
#include <Eigen/Sparse>
#include <Eigen/src/Sparse/SparseMatrix.h>
#include <Eigen/src/Sparse/DynamicSparseMatrix.h>
#include <Eigen/SparseExtra>
#include <Eigen/SVD>

#include <vector>
#include <sys/time.h>

#include "types.h"

typedef Eigen::SparseMatrix<ScalarType> SparseMatrixType;
typedef Eigen::SparseMatrix<ScalarType, Eigen::RowMajor> SparseOrderedMatrixType;


class Deform
{
public:
  
  static void arap(const MatrixXX &original_vertex_positions,
                   const MatrixXXi &faces,
                   MatrixXX &new_vertex_positions,
                   const std::vector <IndexType > &constrained_vertices,
                   const MatrixXX &constrained_vertex_positions,
                   const int maxIter = 10,
                   const float avg_edge = 1.)
  {
    
    if(faces.cols() != 3)
    {
      cerr<< "arap: currently only triangles are supported."<<endl;
      exit(-1);
    }
    
    if (reinitialize)
    {
      igl::cotmatrix( original_vertex_positions, faces, Lap);
      igl::cotangent(original_vertex_positions, faces, cotangents);
      
      igl::vf(original_vertex_positions, faces, vf, vfi);
      igl::adjacency_list(faces, edges);

      reinitialize = false;

    }
    
    long numVertices = original_vertex_positions.rows();
    long numInterior = numVertices - constrained_vertices.size();
    
    ///vertices in constrained_vertices have to be ORDERED !!! 
    if(redoInterior)
    {
      precompute(original_vertex_positions,
                 faces,
                 constrained_vertices,
                 edges);
      
      redoInterior = false;
    }
    
    MatrixXX toSubtractFromRHS = MatrixXX::Zero(numInterior,3);
    for (int i = 0; i<constrained_vertices.size(); ++i)
    {
      size_t ii = constrained_vertices[i];
      for(std::vector<IndexType>::const_iterator it = edges[ii].begin(); it != edges[ii].end(); ++it)
      {
        IndexType j = *it;
        long int jj = indexInInterior[j];
        if(jj>=0)
          toSubtractFromRHS.block<1,3>(jj,0) += Lap.coeff(j,ii)*constrained_vertex_positions.row(i);
      }
    }
    
    
    MatrixXX b(numInterior,3);
    
    MatrixXX R (3*numVertices,3);
    
    MatrixXX T = original_vertex_positions;
    new_vertex_positions = original_vertex_positions;
    
    int iter = 0;
    do
    {
      iter++;
      T = new_vertex_positions;
      
      find_optimal_rotations(new_vertex_positions, Lap, edges, R);
      
      b = LapForRot * R;
      b -= toSubtractFromRHS;
      
      MatrixXX p = solver.solve(b);
      for (int i = 0; i < numInterior; ++i)
        new_vertex_positions.row(indexInBigL[i]) = p.row(i);
      
      for (int i = 0; i < constrained_vertices.size(); ++i)
        new_vertex_positions.row(constrained_vertices[i]) = constrained_vertex_positions.row(i);
    }
    while(iter <maxIter && getMaxVertexShift(T,new_vertex_positions) > 0.001*avg_edge);
    
    
  }
  
  static void find_optimal_rotations(const MatrixXX &goalVertices,  
                                     const SparseMatrixType &Lap,
                                     const std::vector<std::vector<IndexType > > &edges,
                                     MatrixXX &R )
  {
    MatrixXX S = SS * goalVertices;
    
    for(int i = 0; i <goalVertices.rows(); ++i)
    {
      Eigen::JacobiSVD<Matrix33> svd(S.block<3,3>(i*3,0), Eigen::ComputeFullU | Eigen::ComputeFullV );
      
      Matrix33 su = svd.matrixU();
      const Matrix33 sv = svd.matrixV();
      const VectorX ss = svd.singularValues();
      
      Matrix33 mat = sv * su.transpose();
      
      if (mat.determinant() <0)
      {
        su(0,2) = -su(0,2);
        su(1,2) = -su(1,2);
        su(2,2) = -su(2,2);
        
        mat = sv * su.transpose();
      }
      R.block<3,3>(i*3,0) = mat;
    }
  }
  
  
  
  static ScalarType getMaxVertexShift(const MatrixXX &old_vertex_positions, const MatrixXX &new_vertex_positions)
  {
    return (((old_vertex_positions-new_vertex_positions).array().abs()).matrix()).maxCoeff();
  }
  
  static void precompute_indices(const int numInterior,
                                 const int numVertices,
                                 const std::vector <IndexType > &constrained_vertices)
  {
    timeval time;
    double start_seconds, end_seconds;        
    double s, e;        
    gettimeofday(&time, NULL);
    s = time.tv_sec*1000 + time.tv_usec / 1e3;

    indexInBigL.clear();
    indexInBigL.resize(numInterior,-1);
    indexInInterior.clear();
    indexInInterior.resize(numVertices,-1);
    if(numInterior<numVertices)
    {
      long index = 0, vii;
      for (vii = 0; vii<constrained_vertices[0]; indexInInterior[vii] = index, indexInBigL[index++] = vii++);
      
      for (long vi = 1; vi<constrained_vertices.size(); ++vi)
        for (vii = constrained_vertices[vi-1]+1; vii<constrained_vertices[vi]; indexInInterior[vii] = index, indexInBigL[index++] = (vii++));
      for (;index<numInterior;++index)
      {
        indexInBigL[index] = ++vii;
        indexInInterior[vii] = index; 
      }
    }
    else
      for (long vi = 0; vi<numVertices; indexInBigL[vi] = vi++);
    
    gettimeofday(&time, NULL);
    e = time.tv_sec*1000 + time.tv_usec / 1e3;
    printf("%.5f milliseconds for indexing\n",e - s);

  }
  
  static void precompute_interior_laplacian(const int numInterior,
                                            const std::vector<std::vector<IndexType > > &edges)
  {
    timeval time;
    double start_seconds, end_seconds;        
    double s, e;        
    
    gettimeofday(&time, NULL);
    s = time.tv_sec*1000 + time.tv_usec / 1e3;
    
    intLap = SparseOrderedMatrixType (numInterior,numInterior);
    intLap.reserve (numInterior*numInterior);
    for(int i=0; i<numInterior; ++i)
    {
      size_t ii = indexInBigL[i];
      std::vector<IndexType> myedges = edges[ii];
      myedges.push_back(ii);
      std::sort(myedges.begin(),myedges.end());
      intLap.startVec(i);                          // optional for a DynamicSparseMatrix
      for(std::vector<IndexType>::iterator it = myedges.begin(); it != myedges.end(); ++it)
      {
        IndexType j = *it;
        long int jj = indexInInterior[j];
        if (jj>=0)
          intLap.insertBack(i,jj) = Lap.coeff(ii,j);
      }
    }
    intLap.finalize();
    gettimeofday(&time, NULL);
    e = time.tv_sec*1000 + time.tv_usec / 1e3;
    printf("%.5f milliseconds for interior laplacian\n",e - s);
    
    gettimeofday(&time, NULL);
    s = time.tv_sec*1000 + time.tv_usec / 1e3;
    solver = Eigen::SimplicialCholesky<SparseMatrixType, Eigen::RowMajor > (intLap);
    gettimeofday(&time, NULL);
    e = time.tv_sec*1000 + time.tv_usec / 1e3;
    printf("%.5f milliseconds for cholesky\n",e - s);
    
  }
  
  static void precompute_rhs(const MatrixXX &original_vertex_positions,
                             const MatrixXXi &faces,
                             const int numInterior,
                             const std::vector<std::vector<IndexType > > &edges)
  {
    timeval time;
    double start_seconds, end_seconds;        
    double s, e;        
    gettimeofday(&time, NULL);
    s = time.tv_sec*1000 + time.tv_usec / 1e3;
    
    MatrixXX temp;
    int numVertices = original_vertex_positions.rows();
    Eigen::Matrix<int,3,2> tri_edges;
    tri_edges << 1,2, 2,0, 0,1;
    if (!use_spokes_and_rims)
    {
      LapForRot = SparseOrderedMatrixType (numInterior, 3*numVertices);
      LapForRot.reserve (numInterior*7*3);
      temp = MatrixXX::Zero(numInterior,3);
      for (int i=0; i<numInterior; ++i)
      {
        size_t iii = indexInBigL[i];
        LapForRot.startVec(i);
        
        std::vector<IndexType> myedges = edges[iii];
        myedges.push_back(iii);
        std::sort(myedges.begin(),myedges.end());
        
        for(std::vector<IndexType>::iterator it = myedges.begin(); it != myedges.end(); ++it)
        {
          IndexType j = *it;
          IndexType jj = 3*j;
          
          Vector3 v = 0.5*Lap.coeff(iii,j)* (original_vertex_positions.row(j) - original_vertex_positions.row(iii));
          temp.row(i) += v;
          for (int step = 0; step<3; ++step)
            LapForRot.insertBack(i,jj+step) = v(step);
        }
      }
      LapForRot.finalize();
      for(int i=0; i<numInterior; ++i)
      {
        size_t ii = 3*indexInBigL[i];
        for (int step = 0; step<3; ++step)
          LapForRot.coeffRef(i,ii+step) = temp.coeff(i,step);
      }  
    }
    
    else
    {
      Eigen::DynamicSparseMatrix<ScalarType, Eigen::RowMajor> dyn_LapForRot (numInterior, 3*numVertices);
      // This is important! it could decrease the comptuation time by a factor of 2
      // Laplacian for a closed 2d manifold mesh will have on average 7 entries per
      // row
      dyn_LapForRot.reserve(7*numVertices);
      // Loop over triangles
      for(int ii = 0; ii < numInterior; ii++)
      {
        size_t i = indexInBigL[ii];
        const std::vector<IndexType> & myfaces = vf[i];
        const std::vector<IndexType> & myface_indices = vfi[i];
        
        for (int j = 0; j<myfaces.size(); ++j)
        {
          for (int k =0; k<faces.cols(); ++k)
          {
            int current_vertex_index = faces(myfaces[j],k);
            if (current_vertex_index == i)
              continue;
            
            double cot = cotangents(myfaces[j],k);
            int edge_0 = faces(myfaces[j],tri_edges(k,0));
            int edge_1 = faces(myfaces[j],tri_edges(k,1));
            
            if(edge_0 ==i)
            {
              int temp = edge_0;
              edge_0 = edge_1;
              edge_1 = temp;
            }
            Vector3 v = cot* (original_vertex_positions.row(edge_0) - original_vertex_positions.row(edge_1));
            
            for (int step = 0; step<3; ++step)
            {
              dyn_LapForRot.coeffRef(ii,3*edge_0+step) += v[step];
              dyn_LapForRot.coeffRef(ii,3*edge_1+step) += v[step];
              dyn_LapForRot.coeffRef(ii,3*current_vertex_index+step) += v[step];
            }
            
            
          }
          
        }
      }
      LapForRot = SparseMatrixType(dyn_LapForRot)/3;
      
      
    }
    gettimeofday(&time, NULL);
    e = time.tv_sec*1000 + time.tv_usec / 1e3;
    printf("%.5f milliseconds for LapForRot\n",e - s);   
    
  }
  
  
  static void precompute_covariance_matrix(const MatrixXX &original_vertex_positions,
                                           const MatrixXXi &faces,
                                           const std::vector<std::vector<IndexType > > &edges)
  {
    timeval time;
    double start_seconds, end_seconds;        
    double s, e;        
    
    gettimeofday(&time, NULL);
    s = time.tv_sec*1000 + time.tv_usec / 1e3;
    
    int numVertices = original_vertex_positions.rows();
    MatrixXX temp;
    Eigen::Matrix<int,3,2> tri_edges;
    tri_edges << 1,2, 2,0, 0,1;
    
    if(!use_spokes_and_rims)
    {
      SS = SparseOrderedMatrixType (3*numVertices, numVertices);
      SS.reserve (numVertices*7*3);
      temp = MatrixXX::Zero(numVertices,3);
      for(int i=0; i<numVertices; ++i)
      {
        size_t ii = 3*i;
        std::vector<IndexType> myedges = edges[i];
        VectorX weights = VectorX::Zero(myedges.size());
        
        for(std::vector<IndexType>::iterator it = myedges.begin(); it != myedges.end(); ++it)
          weights[it - myedges.begin()] = std::max(ScalarType(0.),Lap.coeff(i,*it));
        weights /= weights.sum();
        
        myedges.push_back(i);
        std::sort(myedges.begin(),myedges.end());
        
        for (int step = 0; step<3; ++step)
        {
          SS.startVec(ii+step);
          for(std::vector<IndexType>::iterator it = myedges.begin(); it != myedges.end(); ++it)
          {
            IndexType j = *it;
            Vector3 v = weights[it-myedges.begin()]* (original_vertex_positions.row(j) - original_vertex_positions.row(i));
            SS.insertBack(ii+step,j) = v(step);
            if(step == 0)
              temp.row(i) -= v;
          }
        }
      }
      SS.finalize();
      for(int i=0; i<numVertices; ++i)
      {
        size_t ii = 3*i;
        for (int step = 0; step<3; ++step)
          SS.coeffRef(ii+step,i) = temp.coeff(i,step);
      }
    }
    else
    {
      Eigen::DynamicSparseMatrix<ScalarType, Eigen::RowMajor> dyn_SS (3*numVertices, numVertices);
      // This is important! it could decrease the comptuation time by a factor of 2
      // Laplacian for a closed 2d manifold mesh will have on average 7 entries per
      // row
      dyn_SS.reserve(numVertices*7*3);
      // Loop over triangles
      // Loop over triangles
      for(int i = 0; i < numVertices; i++)
      {
        const std::vector<IndexType> & myfaces = vf[i];
        const std::vector<IndexType> & myface_indices = vfi[i];
        
        for (int j = 0; j<myfaces.size(); ++j)
        {
          for (int k =0; k<faces.cols(); ++k)
          {
            int current_vertex_index = faces(myfaces[j],k);
            
            double cot = cotangents(myfaces[j],k);
            int edge_0 = faces(myfaces[j],tri_edges(k,0));
            int edge_1 = faces(myfaces[j],tri_edges(k,1));
            
            Vector3 v = cot* (original_vertex_positions.row(edge_0) - original_vertex_positions.row(edge_1));
            
            for (int step = 0; step<3; ++step)
            {
              dyn_SS.coeffRef(3*i+step,edge_0) += v[step];
              dyn_SS.coeffRef(3*i+step,edge_1) += -v[step];
            }
            
            
          }
          
        }
      }
      // Corner indices of this triangle
      SS = SparseMatrixType(dyn_SS);
      
      
    }
    gettimeofday(&time, NULL);
    e = time.tv_sec*1000 + time.tv_usec / 1e3;
    printf("%.5f milliseconds for SS\n",e - s);
  }
  
  
  static void precompute(const MatrixXX &original_vertex_positions,
                         const MatrixXXi &faces,
                         const std::vector <IndexType > &constrained_vertices,
                         const std::vector<std::vector<IndexType > > &edges)
  {
    Eigen::Matrix<int,3,2> tri_edges;
    tri_edges << 1,2, 2,0, 0,1;
    long numVertices = original_vertex_positions.rows();
    long numInterior = numVertices - constrained_vertices.size();


    precompute_indices(numInterior,
                       numVertices,
                       constrained_vertices);

    precompute_interior_laplacian(numInterior,
                                  edges);
    
    precompute_rhs(original_vertex_positions,
                   faces,
                   numInterior,
                   edges);

    precompute_covariance_matrix(original_vertex_positions,
                                 faces,
                                 edges);
    
  }
  
  static std::vector<long > indexInBigL;
  static std::vector<long > indexInInterior;
  
  static SparseOrderedMatrixType intLap;
  
  static SparseOrderedMatrixType LapForRot;
  
  static bool redoInterior;
  static bool reinitialize;
  
  static bool use_simple_rhs;
  static bool use_spokes_and_rims;

  static Eigen::SimplicialCholesky<SparseMatrixType, Eigen::RowMajor > solver;
  
  static SparseOrderedMatrixType SS;
  
  static SparseMatrixType Lap;

  static MatrixXX cotangents;
  static std::vector<std::vector<IndexType> >vf;
  static std::vector<std::vector<IndexType> > vfi;
  static std::vector<std::vector<IndexType > > edges;
  

};

std::vector<long > Deform::indexInBigL = std::vector<long> (0);
std::vector<long > Deform::indexInInterior = std::vector<long> (0);
SparseOrderedMatrixType Deform::intLap = SparseOrderedMatrixType(0,0);
SparseOrderedMatrixType Deform::LapForRot = SparseOrderedMatrixType(0,0);
SparseOrderedMatrixType Deform::SS = SparseOrderedMatrixType(0,0);
SparseMatrixType Deform::Lap = SparseMatrixType(0,0);
;

MatrixXX Deform::cotangents = MatrixXX(0,0);
std::vector<std::vector<IndexType> > Deform::vf = std::vector<std::vector<IndexType> >(0);
std::vector<std::vector<IndexType> > Deform::vfi = std::vector<std::vector<IndexType> >(0);
std::vector<std::vector<IndexType> > Deform::edges = std::vector<std::vector<IndexType> >(0);

bool Deform::redoInterior = true;
bool Deform::reinitialize = true;
bool Deform::use_simple_rhs = false;
bool Deform::use_spokes_and_rims = true;
Eigen::SimplicialCholesky<SparseMatrixType, Eigen::RowMajor > Deform::solver(intLap);

#endif
