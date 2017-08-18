// Copyright 2013 - Christian Schüller 2013, schuellc@inf.ethz.ch
// Interactive Geometry Lab - ETH Zurich

#include <iostream>
#include <math.h>
#include "mex.h"
#include <vector>
#include <fstream>

#define MATLAB_LINK

#include "LIMSolverInterface.h"

#include <Eigen/Core>

using namespace std;
using namespace Eigen;
ofstream* os;

extern void _main();

// Data stored between different calls
LIMData* data = NULL;

template<class T>
vector<vector<T> > readMatrix(const mxArray* mat)
{
  T* ptr = mxGetPr(mat);
        
  int m = mxGetM(mat);
  int n = mxGetN(mat);
    
  vector<vector<T> >	V;
  V.resize(m);
    for(int j=0;j<n;j++)
      for(int i=0;i<m;++i)
        V[i].push_back(*(ptr++));
        
  return V;

} 

template<class T>
mxArray* writeMatrix(vector<vector<T> > mat)
{
  mxArray* ret = 0;

  if (mat.size() == 0)
  {
    ret =	mxCreateNumericMatrix(0, 0, mxDOUBLE_CLASS, mxREAL);
  }
  else
  {
    int M = mat.size();
    int N = mat[0].size();

    ret = mxCreateNumericMatrix(M, N, mxDOUBLE_CLASS, mxREAL);
    double* pointer = mxGetPr(ret);

    /* Copy data into the mxArray */
    int count = 0;
    for ( int j = 0; j < N; ++j ) 
    {
      for (int i = 0; i < M; ++i)
      {
        pointer[count++] = mat[i][j];
      }
    }
  }

  return ret;
}

template<class T>
void plotMatrix(vector<vector<T> >& M, const char* str)
{

  cout << str << ":" << endl;
        
  for (int i = 0; i < M.size(); ++i)
  {
    for (int j = 0; j < M[i].size(); ++j)
    {
      cout << M[i][j] << " ";
    }
    cout << endl;
  }
} 

template<class T>
Matrix<T,Dynamic,Dynamic> VV2Matrix(const vector<vector< T > >& vv, int cols)
{
  Matrix<T,Dynamic,Dynamic> M(vv.size(),cols);

  for (unsigned i=0; i<vv.size(); ++i)
  {
    assert(vv[i].size() == cols);
    for (unsigned j=0; j<cols;++j)
      M(i,j) = vv[i][j];
  }
  return M;
}

template<class T, int C>
Matrix<T,Dynamic,C> VV2Matrix(const vector<vector< T > >& vv)
{
  Matrix<T,Dynamic,C> M(vv.size(),C);

  for (unsigned i=0; i<vv.size(); ++i)
  {
    assert(vv[i].size() == C);
    for (unsigned j=0; j<C;++j)
      M(i,j) = vv[i][j];
  }
  return M;
}

template<class T>
vector<vector< T > > Matrix2VV(const Matrix<T,Dynamic,Dynamic>& M)
{
  vector< vector< T > > vv;
    for (unsigned j=0; j<M.rows(); ++j)
  {
    vector<T> t(M.cols);
        for (unsigned i=0; i<M.cols;++i)
            t[i] = M(j,i);
    vv.push_back(t);
  }
  return vv;
}

template<class T, int C>
vector<vector< T > > Matrix2VV(const Matrix<T,Dynamic,C>& M)
{
  assert(M.cols() == C);
  
  vector< vector< T > > vv;
    for (unsigned j=0; j<M.rows(); ++j)
  {
    vector<T> t(C);
        for (unsigned i=0; i<C;++i)
            t[i] = M(j,i);
    vv.push_back(t);
  }
  return vv;
}

void mexFunction(
     int          nlhs,
     mxArray      *plhs[],
     int          nrhs,
     const mxArray *prhs[]
     )
{
  /* Check for proper number of arguments */

  if (nrhs != 1 && nrhs != 12)
  {
    mexErrMsgIdAndTxt("MATLAB:mexcpp:nargin", 
      "lim_solver_mex requires 1 or 12 input arguments.");
  } 

  if(nrhs == 12)
  {
    const mxArray* vertices_mx = prhs[0];
    const mxArray* initialVertices_mx = prhs[1];
    const mxArray* elements_mx = prhs[2];
    const mxArray* borderVertices_mx = prhs[3];
    const mxArray* gradients_mx = prhs[4];
    const mxArray* constraintMatrix_mx = prhs[5];
    const mxArray* constraintTargets_mx = prhs[6];
    const mxArray* energyType_mx = prhs[7];
    const mxArray* enableOuput_mx = prhs[8];
    const mxArray* enableAlphaUpdate_mx = prhs[9];
    const mxArray* beta_mx = prhs[10];
    const mxArray* eps_mx = prhs[11];

    // os = new ofstream("/Users/daniele/log.txt");

    vector<vector<double> > vertices_VV = readMatrix<double>(vertices_mx);
    vector<vector<double> > initialVertices_VV = readMatrix<double>(initialVertices_mx);
    vector<vector<double> > elements_VV = readMatrix<double>(elements_mx);
    vector<vector<double> > borderVertices_VV = readMatrix<double>(borderVertices_mx);
    vector<vector<double> > gradients_VV = readMatrix<double>(gradients_mx);
    vector<vector<double> > constraintMatrix_VV = readMatrix<double>(constraintMatrix_mx);
    vector<vector<double> > constraintTargets_VV = readMatrix<double>(constraintTargets_mx);
    vector<vector<double> > energyType_VV = readMatrix<double>(energyType_mx);
    vector<vector<double> > enableOuput_VV = readMatrix<double>(enableOuput_mx);
    vector<vector<double> > enableAlphaUpdate_VV = readMatrix<double>(enableAlphaUpdate_mx);
    vector<vector<double> > beta_VV = readMatrix<double>(beta_mx);
    vector<vector<double> > eps_VV = readMatrix<double>(eps_mx);
  
    bool isTetMesh = (elements_VV[0].size() == 4);

    Matrix<double,Dynamic,3> vertices = VV2Matrix<double,3>(vertices_VV);
    Matrix<double,Dynamic,3> initialVertices = VV2Matrix<double,3>(initialVertices_VV);
  
    Matrix<int,Dynamic,Dynamic> elements;
    if(isTetMesh)
      elements = VV2Matrix(elements_VV,4).cast<int>();
    else
      elements = VV2Matrix(elements_VV,3).cast<int>();
    elements = elements.array() - 1;
  
    vector<int> borderVertices;
    if (borderVertices_VV.size() != 0)
    {
      if(borderVertices_VV.size() == 1)
      {
      borderVertices = vector<int>(borderVertices_VV[0].begin(),borderVertices_VV[0].end());
      for (unsigned i=0; i<borderVertices.size();++i)
        borderVertices[i] = borderVertices[i]-1;
      }
      else
      {
        cerr << "borderVertices should be a row vector!" << endl;
        exit(0);
      }
    }
    Matrix<double,Dynamic,1> gradients = VV2Matrix<double,1>(gradients_VV);
  
    Matrix<double,Dynamic,3> constraintTriplets = VV2Matrix<double,3>(constraintMatrix_VV);
    vector<Triplet<double> > triplets;
    int maxRow = -1;
    for(int i=0;i<constraintTriplets.rows();i++)
    {
      int row = (int)constraintTriplets(i,0)-1;
      int col = (int)constraintTriplets(i,1)-1;
      double val = constraintTriplets(i,2);

      if(maxRow < row)
        maxRow = row;

      triplets.push_back(Triplet<double>(row,col,val));
    }
    SparseMatrix<double> constraintMatrix;

    if(isTetMesh)
      constraintMatrix.resize(maxRow+1,initialVertices.rows()*3);
    else
      constraintMatrix.resize(maxRow+1,initialVertices.rows()*2);
    constraintMatrix.setFromTriplets(triplets.begin(),triplets.end());

    Matrix<double,Dynamic,1> constraintTargets = VV2Matrix<double,1>(constraintTargets_VV);

    int energyType = energyType_VV[0][0];
    bool enableOuput = enableOuput_VV[0][0];
    bool enableAlphaUpdate = enableAlphaUpdate_VV[0][0];
    double beta = beta_VV[0][0];
    double eps = eps_VV[0][0];

    if(false)
    {
      cerr << "Vertices:" << endl;
      cerr << vertices << endl;
      cerr << "---------" << endl;
      cerr << "initialVertices:" << endl;
      cerr << initialVertices << endl;
      cerr << "---------" << endl;
      cerr << "elements:" << endl;
      cerr << elements << endl;
      cerr << "---------" << endl;
      cerr << "borderVertices:" << endl;
      for (unsigned i=0; i<borderVertices.size(); ++i)
        cerr << borderVertices[i] << " ";
      cerr << endl;
      cerr << "---------" << endl;
      cerr << "gradients:" << endl;
      cerr << gradients << endl;
      cerr << "---------" << endl;
      cerr << "constraintMatrix:" << endl;
      Matrix<double,Dynamic,Dynamic> M(constraintMatrix);
      cerr << M << endl;
      cerr << "---------" << endl;
      cerr << "constraintTargets:" << endl;
      cerr << constraintTargets << endl;
      cerr << "---------" << endl;
      cerr << "energyType:" << endl;
      cerr << energyType << endl;
      cerr << "---------" << endl;
      cerr << "enableOutput" << endl;
      cerr << enableOuput << endl;
      cerr << "---------" << endl;
      cerr << "enableAlphaUpdate:" << endl;
      cerr << enableAlphaUpdate << endl;
      cerr << "---------" << endl;
      cerr << "beta:" << endl;
      cerr << beta << endl;
      cerr << "---------" << endl;
      cerr << "eps:" << endl;
      cerr << eps << endl;
      cerr << "---------" << endl;
    }

    delete data;

    data = InitLIM(
      vertices,
      initialVertices,
      elements,
      borderVertices,
      gradients,
      constraintMatrix,
      constraintTargets,
      energyType,
      enableOuput,
      enableAlphaUpdate,
      beta,
      eps);
  }
  else
  {
    if (nlhs != 1)
    {
      mexErrMsgIdAndTxt("MATLAB:mexcpp:nargout",
        "lim_solver_mex generates one output argument.");
    }

    const mxArray* vertices_mx = prhs[0];
    vector<vector<double> > vertices_VV = readMatrix<double>(vertices_mx);
    Matrix<double,Dynamic,3> vertices = VV2Matrix<double,3>(vertices_VV);

    int res = ComputeLIM_Step(data, vertices);

    //cerr << "Returned flag: " << r << endl;
    vector<vector<double> > out_VV = Matrix2VV<double,3>(vertices);
    plhs[0] = writeMatrix(out_VV);
  }

  return;
}
