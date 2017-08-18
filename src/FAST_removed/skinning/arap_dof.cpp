#ifndef IGL_HEADER_ONLY
#define IGL_HEADER_ONLY
#endif

#define VERBOSE
//#define EXTREME_VERBOSE

#include "arap_dof.h"
#include "group_sum_matrix.h"
#include "covariance_scatter_matrix.h"
#include "arap_rhs.h"
//#define FIT_ROTATIONS_ALLOW_FLIPS 
//#define ARAP_DOF_FIX_FLIPS
#include "fit_rotations.h"
#include "columnize.h"
#include "uncolumnize.h"
#include "intersection.h"

#include <igl/cotmatrix.h>
#include <igl/speye.h>
#include <igl/repdiag.h>
#include <igl/repmat.h>
#include <igl/min_quad_with_fixed.h>
#include <igl/slice.h>
#include <igl/colon.h>
#include <igl/full.h>
#include <igl/mode.h>
#include <igl/is_symmetric.h>

#include <igl/verbose.h>
#include <igl/print_ijv.h>

#include <igl/get_seconds_hires.h>
//#include "MKLEigenInterface.h"//wangyu
#include <igl/writeDMAT.h>
#include <igl/min_quad_dense.h>
#include <igl/get_seconds.h>

using namespace std;
using namespace igl;
using namespace Eigen;

// defined if no early exit is supported, i.e., always take a fixed number of iterations
#define FIXED_ITERATIONS_COUNT

//#define MATLAB_DEBUG
#ifdef MATLAB_DEBUG
	#include <matlabinterface.h>
	Engine *g_pEngine;
#endif

#ifdef __APPLE__
#  define USE_LU_DECOMPOSITION
#endif

// A carefull derivation of this implementation is given in the corresponding
// matlab function arap_dof.m
bool arap_dof_precomputation(
  const Eigen::MatrixXd & V, 
  const Eigen::MatrixXi & F,
  const Eigen::SparseMatrix<double> & M,
  const Eigen::Matrix<int,Eigen::Dynamic,1> & G,
  const Eigen::Matrix<int,Eigen::Dynamic,1> & fixed,
  const Eigen::SparseMatrix<double> & A_eq,
  arap_dof_data & data)
{
  // number of mesh (domain) vertices
  int n = V.rows();
  // cache problem size
  data.n = n;
  // dimension of mesh
  data.dim = V.cols();
  assert(data.dim == M.rows()/n);
  assert(data.dim*n == M.rows());
  if(data.dim == 3)
  {
    // Check if z-coordinate is all zeros
    if(V.col(2).minCoeff() == 0 && V.col(2).maxCoeff() == 0)
    {
      data.effective_dim = 2;
    }
  }else
  {
    data.effective_dim = data.dim;
  }
  // Number of handles
  data.m = M.cols()/data.dim/(data.dim+1);
  assert(data.m*data.dim*(data.dim+1) == M.cols());
  //assert(m == C.rows());

  verbose("n=%d; dim=%d; m=%d;\n",n,data.dim,data.m);

  // Build cotangent laplacian
  SparseMatrix<double> Lcot;
  verbose("cotmatrix()\n");
  cotmatrix(V,F,Lcot);
  // Discrete laplacian (should be minus matlab version)
  SparseMatrix<double> Lapl = -2.0*Lcot;
#ifdef EXTREME_VERBOSE
  cout<<"LaplIJV=["<<endl;print_ijv(Lapl,1);cout<<endl<<"];"<<
    endl<<"Lapl=sparse(LaplIJV(:,1),LaplIJV(:,2),LaplIJV(:,3),"<<
    Lapl.rows()<<","<<Lapl.cols()<<");"<<endl;
#endif

  // Get group sum scatter matrix, when applied sums all entries of the same
  // group according to G
  SparseMatrix<double> G_sum;
  if(G.size() == 0)
  {
    speye(n,G_sum);
  }else
  {
    // groups are defined per vertex, convert to per face using mode
    Eigen::Matrix<int,Eigen::Dynamic,1> GG;
    if(data.energy == ARAP_ELEMENTS)
    {
      MatrixXi GF(F.rows(),F.cols());
      for(int j = 0;j<F.cols();j++)
      {
        Matrix<int,Eigen::Dynamic,1> GFj;
        slice(G,F.col(j),GFj);
        GF.col(j) = GFj;
      }
      mode<int>(GF,2,GG);
    }else
    {
      GG=G;
    }

    verbose("group_sum_matrix()\n");
    group_sum_matrix(GG,GG.maxCoeff()+1,G_sum);
  }
#ifdef EXTREME_VERBOSE
  cout<<"G_sumIJV=["<<endl;print_ijv(G_sum,1);cout<<endl<<"];"<<
    endl<<"G_sum=sparse(G_sumIJV(:,1),G_sumIJV(:,2),G_sumIJV(:,3),"<<
    G_sum.rows()<<","<<G_sum.cols()<<");"<<endl;
#endif

  // Get covariance scatter matrix, when applied collects the covariance matrices
  // used to fit rotations to during optimization
  SparseMatrix<double> CSM;
  verbose("covariance_scatter_matrix()\n");
  covariance_scatter_matrix(V,F,data.energy,CSM);
#ifdef EXTREME_VERBOSE
  cout<<"CSMIJV=["<<endl;print_ijv(CSM,1);cout<<endl<<"];"<<
    endl<<"CSM=sparse(CSMIJV(:,1),CSMIJV(:,2),CSMIJV(:,3),"<<
    CSM.rows()<<","<<CSM.cols()<<");"<<endl;
#endif
  

  // Build the covariance matrix "constructor". This is a set of *scatter*
  // matrices that when multiplied on the right by column of the transformation
  // matrix entries (the degrees of freedom) L, we get a stack of dim by 1
  // covariance matrix column, with a column in the stack for each rotation
  // *group*. The output is a list of matrices because we construct each column
  // in the stack of covariance matrices with an independent matrix-vector
  // multiplication.
  //
  // We want to build S which is a stack of dim by dim covariance matrices.
  // Thus S is dim*g by dim, where dim is the number of dimensions and g is the
  // number of groups. We can precompute dim matrices CSM_M such that column i
  // in S is computed as S(:,i) = CSM_M{i} * L, where L is a column of the
  // skinning transformation matrix values. To be clear, the covariance matrix
  // for group k is then given as the dim by dim matrix pulled from the stack:
  // S((k-1)*dim + 1:dim,:)

  // Apply group sum to each dimension's block of covariance scatter matrix
  SparseMatrix<double> G_sum_dim;
  repdiag(G_sum,data.dim,G_sum_dim);
  CSM = G_sum_dim * CSM;
#ifdef EXTREME_VERBOSE
  cout<<"CSMIJV=["<<endl;print_ijv(CSM,1);cout<<endl<<"];"<<
    endl<<"CSM=sparse(CSMIJV(:,1),CSMIJV(:,2),CSMIJV(:,3),"<<
    CSM.rows()<<","<<CSM.cols()<<");"<<endl;
#endif

  verbose("CSM_M()\n");
  // Precompute CSM times M for each dimension
  data.CSM_M.resize(data.dim);
#ifdef EXTREME_VERBOSE
  cout<<"data.CSM_M = cell("<<data.dim<<",1);"<<endl;
#endif
  // span of integers from 0 to n-1
  Eigen::Matrix<int,Eigen::Dynamic,1> span_n(n);
  for(int i = 0;i<n;i++)
  {
    span_n(i) = i;
  }

  // span of integers from 0 to M.cols()-1
  Eigen::Matrix<int,Eigen::Dynamic,1> span_mlbs_cols(M.cols());
  for(int i = 0;i<M.cols();i++)
  {
    span_mlbs_cols(i) = i;
  }

  // number of groups
  int k = CSM.rows()/data.dim;
  for(int i = 0;i<data.dim;i++)
  {
    verbose("CSM_M(): Mi\n");
    SparseMatrix<double> M_i;
    verbose("CSM_M(): slice\n");
    slice(M,(span_n.array()+i*n).matrix(),span_mlbs_cols,M_i);
    SparseMatrix<double> M_i_dim;
    data.CSM_M[i].resize(k*data.dim,data.m*data.dim*(data.dim+1));
    assert(data.CSM_M[i].cols() == M.cols());
    for(int j = 0;j<data.dim;j++)
    {
      SparseMatrix<double> CSMj;
      verbose("CSM_M(): slice\n");
      slice(
        CSM,
        colon<int>(j*k,(j+1)*k-1),
        colon<int>(j*n,(j+1)*n-1),
        CSMj);
      assert(CSMj.rows() == k);
      assert(CSMj.cols() == n);
      SparseMatrix<double> CSMjM_i = CSMj * M_i;
      MatrixXd CSMjM_ifull;
      verbose("CSM_M(): full\n");
      full(CSMjM_i,CSMjM_ifull);
      data.CSM_M[i].block(j*k,0,CSMjM_i.rows(),CSMjM_i.cols()) = CSMjM_ifull;
    }
#ifdef EXTREME_VERBOSE
    cout<<"CSM_Mi=["<<endl<<data.CSM_M[i]<<endl<<"];"<<endl;
#endif
  }

  // precompute arap_rhs matrix
  verbose("arap_rhs()\n");
  SparseMatrix<double> K;
  arap_rhs(V,F,data.energy,K);
#ifdef EXTREME_VERBOSE
  cout<<"KIJV=["<<endl;print_ijv(K,1);cout<<endl<<"];"<<
    endl<<"K=sparse(KIJV(:,1),KIJV(:,2),KIJV(:,3),"<<
    K.rows()<<","<<K.cols()<<");"<<endl;
#endif
  // Precompute left muliplication by M and right multiplication by G_sum
  SparseMatrix<double> G_sumT = G_sum.transpose();
  SparseMatrix<double> G_sumT_dim_dim;
  repdiag(G_sumT,data.dim*data.dim,G_sumT_dim_dim);
  SparseMatrix<double> MT = M.transpose();
  // If this is a bottle neck then consider reordering matrix multiplication
  data.M_KG = -4.0 * MT * K * G_sumT_dim_dim;
#ifdef EXTREME_VERBOSE
  cout<<"data.M_KGIJV=["<<endl;print_ijv(data.M_KG,1);cout<<endl<<"];"<<
    endl<<"data.M_KG=sparse(data.M_KGIJV(:,1),data.M_KGIJV(:,2),data.M_KGIJV(:,3),"<<
    data.M_KG.rows()<<","<<data.M_KG.cols()<<");"<<endl;
#endif

  // Precompute system matrix
  verbose("A()\n");
  SparseMatrix<double> A;
  repdiag(Lapl,data.dim,A);
  data.Q = MT * A * M;
#ifdef EXTREME_VERBOSE
  cout<<"QIJV=["<<endl;print_ijv(data.Q,1);cout<<endl<<"];"<<
    endl<<"Q=sparse(QIJV(:,1),QIJV(:,2),QIJV(:,3),"<<
    data.Q.rows()<<","<<data.Q.cols()<<");"<<endl;
#endif

  // This may/should be superfluous
  verbose("is_symmetric()\n");
  if(!is_symmetric(data.Q))
  {
    // "Fix" symmetry
    SparseMatrix<double> QT = data.Q.transpose();
    SparseMatrix<double> Q_copy = data.Q;
    data.Q = 0.5*(Q_copy+QT);
    // Check that ^^^ this really worked
    assert(is_symmetric(data.Q));
  }

  verbose("arap_dof_precomputation() succeeded... so far...\n");
  printf("Number of handles: %i\n", data.m);
  // Finish with "recomputation"
  return arap_dof_recomputation(fixed,A_eq,data);
}

// returns maximal difference of 'blok' from scalar times 3x3 identity:
static float maxBlokErr(const Matrix3f &blok)
{
	float mD;
	float value = blok(0,0);
	float diff1 = fabs(blok(1,1) - value);
	float diff2 = fabs(blok(2,2) - value);
	if (diff1 > diff2) mD = diff1;
	else mD = diff2;

	for (int v=0; v<3; v++)
	{
		for (int w=0; w<3; w++)
		{
			if (v == w) continue;
			if (mD < fabs(blok(v, w))) mD = fabs(blok(v, w));
		}
	}

	return mD;
}

// converts CSM_M_float[0], CSM_M_float[1], CSM_M_float[2] into one "condensed" matrix CSM while checking we're not 
// loosing any information by this process; specifically, returns maximal difference from scaled 3x3 identity blocks,
// which should be pretty small number
static float condense_CSM(const std::vector<MatrixXf> &CSM_M_float, int numBones, int dim, MatrixXf &CSM)
{
	const int numRows = CSM_M_float[0].rows();
	assert(CSM_M_float[0].cols() == dim*(dim+1)*numBones);
	assert(CSM_M_float[1].cols() == dim*(dim+1)*numBones);
	assert(CSM_M_float[2].cols() == dim*(dim+1)*numBones);
	assert(CSM_M_float[1].rows() == numRows);
	assert(CSM_M_float[2].rows() == numRows);

	const int numCols = (dim + 1)*numBones;
	CSM.resize(numRows, numCols);

	float maxDiff = 0.0f;

	for (int r=0; r<numRows; r++)
	{
		for (int coord=0; coord<dim+1; coord++)
		{
			for (int b=0; b<numBones; b++)
			{
				// this is just a test if we really have a multiple of 3x3 identity
				Matrix3f blok;
				for (int v=0; v<3; v++)
				{
					for (int w=0; w<3; w++)
					{
						blok(v,w) = CSM_M_float[v](r, coord*(numBones*dim) + b + w*numBones);
					}					
				}

				//float value[3];
				//for (int v=0; v<3; v++)
				//	CSM_M_float[v](r, coord*(numBones*dim) + b + v*numBones);

				float mD = maxBlokErr(blok);
				if (mD > maxDiff) maxDiff = mD;

				// use the first value:
				CSM(r, coord*numBones + b) = blok(0,0);
			}
		}
	}

	return maxDiff;
}

// splits x_0, ... , x_dim coordinates in column vector 'L' into a numBones*(dimp1) x dim matrix 'Lsep';
// assumes 'Lsep' has already been preallocated
static void splitColumns(const MatrixXf &L, int numBones, int dim, int dimp1, MatrixXf &Lsep)
{
	assert(L.cols() == 1);
	assert(L.rows() == dim*(dimp1)*numBones);

	assert(Lsep.rows() == (dimp1)*numBones && Lsep.cols() == dim);

	for (int b=0; b<numBones; b++)
	{
		for (int coord=0; coord<dimp1; coord++)
		{
			for (int c=0; c<dim; c++)
			{
				Lsep(coord*numBones + b, c) = L(coord*numBones*dim + c*numBones + b, 0);
			}
		}
	}
}


// the inverse of splitColumns, i.e., takes numBones*(dimp1) x dim matrix 'Lsep' and merges the dimensions
// into columns vector 'L' (which is assumed to be already allocated):
static void mergeColumns(const MatrixXf &Lsep, int numBones, int dim, int dimp1, MatrixXf &L)
{
	assert(L.cols() == 1);
	assert(L.rows() == dim*(dimp1)*numBones);

	assert(Lsep.rows() == (dimp1)*numBones && Lsep.cols() == dim);

	for (int b=0; b<numBones; b++)
	{
		for (int coord=0; coord<dimp1; coord++)
		{
			for (int c=0; c<dim; c++)
			{
				L(coord*numBones*dim + c*numBones + b, 0) = Lsep(coord*numBones + b, c);
			}
		}
	}
}

// converts "Solve1" the "rotations" part of FullSolve matrix (the first part) into one "condensed" matrix CSolve1 while checking we're not 
// loosing any information by this process; specifically, returns maximal difference from scaled 3x3 identity blocks,
// which should be pretty small number
static float condense_Solve1(MatrixXf &Solve1, int numBones, int numGroups, int dim, MatrixXf &CSolve1)
{
	assert(Solve1.rows() == dim*(dim + 1)*numBones);
	assert(Solve1.cols() == dim*dim*numGroups);

	float maxDiff = 0.0f;

	CSolve1.resize((dim + 1)*numBones, dim*numGroups);	
	for (int rowCoord=0; rowCoord<dim+1; rowCoord++)
	{
		for (int b=0; b<numBones; b++)
		{
			for (int colCoord=0; colCoord<dim; colCoord++)
			{
				for (int g=0; g<numGroups; g++)
				{
					Matrix3f blok;
					for (int r=0; r<3; r++)
					{
						for (int c=0; c<3; c++)
						{
							blok(r, c) = Solve1(rowCoord*numBones*dim + r*numBones + b, colCoord*numGroups*dim + c*numGroups + g);
						}
					}

					float mD = maxBlokErr(blok);
					if (mD > maxDiff) maxDiff = mD;

					CSolve1(rowCoord*numBones + b, colCoord*numGroups + g) = blok(0,0);
				}
			}
		}
	}	
	
	return maxDiff;
}

bool arap_dof_recomputation(
  const Eigen::Matrix<int,Eigen::Dynamic,1> & fixed,
  const Eigen::SparseMatrix<double> & A_eq,
  arap_dof_data & data)
{
		double time_start = get_seconds();

  assert((int)data.CSM_M.size() == data.dim);
  assert(A_eq.cols() == data.m*data.dim*(data.dim+1));
  if(fixed.size() > 0)
  {
    assert(fixed.maxCoeff() < data.m);
    assert(fixed.minCoeff() >= 0);
  }
  
  // Gather list of indices to known values (each dimension of fixed)
  data.fixed_dim.resize(fixed.size()*data.dim*(data.dim+1));
  for(int d = 0;d<data.dim*(data.dim+1);d++)
  {
    for(int i = 0;i<(int)fixed.size();i++)
    {
      data.fixed_dim(fixed.size()*d + i) = d*(data.m)+fixed(i);
    }
  }
//#ifdef EXTREME_VERBOSE
  cout<<"data.fixed_dim=["<<endl<<data.fixed_dim<<endl<<"]+1;"<<endl;
//#endif

  // Compute dense solve matrix (alternative of matrix factorization)
  verbose("min_quad_dense_precompute()\n");
  MatrixXd Qfull; full(data.Q, Qfull);  
  MatrixXd A_eqfull; full(A_eq, A_eqfull);
  MatrixXd M_Solve;

  double timer0_start = get_seconds_hires();
  min_quad_dense_precompute(Qfull, A_eqfull, true, M_Solve);
  double timer0_end = get_seconds_hires();
  printf("Bob timing: %.20f\n", (timer0_end - timer0_start)*1000.0);

  // Precompute full solve matrix:
  const int fsRows = data.m * data.dim * (data.dim + 1); // 12 * number_of_bones
  const int fsCols1 = data.M_KG.cols(); // 9 * number_of_posConstraints
  const int fsCols2 = A_eq.rows(); // number_of_posConstraints
  data.M_FullSolve.resize(fsRows, fsCols1 + fsCols2);
  // note the magical multiplicative constant "-0.5", I've no idea why it has
  // to be there :)
  data.M_FullSolve << 
    (-0.5 * M_Solve.block(0, 0, fsRows, fsRows) * data.M_KG).cast<float>(), 
    M_Solve.block(0, fsRows, fsRows, fsCols2).cast<float>();

  // Precompute condensed matrices,
  // first CSM:
  std::vector<MatrixXf> CSM_M_float;
  CSM_M_float.resize(data.dim);
  for (int i=0; i<data.dim; i++) CSM_M_float[i] = data.CSM_M[i].cast<float>();
  float maxErr1 = condense_CSM(CSM_M_float, data.m, data.dim, data.CSM);  
  printf("condense_CSM maxErr = %.15f (this should be close to zero)\n", maxErr1);
  assert(fabs(maxErr1) < 1e-5);
  
  // and then solveBlock1:
  // number of groups
  const int k = data.CSM_M[0].rows()/data.dim;
  MatrixXf SolveBlock1 = data.M_FullSolve.block(0, 0, data.M_FullSolve.rows(), data.dim * data.dim * k);
  float maxErr2 = condense_Solve1(SolveBlock1, data.m, k, data.dim, data.CSolveBlock1);  
  printf("condense_Solve1 maxErr = %.15f (this should be close to zero)\n", maxErr2);
  assert(fabs(maxErr2) < 1e-5);

  verbose("arap_dof_recomputation() succeeded, time = %f\n", get_seconds() - time_start);
  return true;
}

bool arap_dof_update(
  const arap_dof_data & data,
  const Eigen::Matrix<double,Eigen::Dynamic,1> & B_eq,
  const Eigen::MatrixXd & L0,
  const int max_iters,
  const double tol,
  Eigen::MatrixXd & L
  )
{
#ifdef ARAP_GLOBAL_TIMING
	double timer_start = get_seconds_hires();
#endif

  // number of dimensions
  assert((int)data.CSM_M.size() == data.dim);
  assert((int)L0.size() == (data.m)*data.dim*(data.dim+1));
  assert(max_iters >= 0);
  assert(tol >= 0);

  // timing variables
  double 
    sec_start, 
    sec_covGather, 
    sec_fitRotations, 
    //sec_rhs, 
    sec_prepMult, 
    sec_solve, sec_end;

  assert(L0.cols() == 1);
#ifdef EXTREME_VERBOSE
  cout<<"dim="<<data.dim<<";"<<endl;
  cout<<"m="<<data.m<<";"<<endl;
#endif

  // number of groups
  const int k = data.CSM_M[0].rows()/data.dim;
  for(int i = 0;i<data.dim;i++)
  {
    assert(data.CSM_M[i].rows()/data.dim == k);
  }
#ifdef EXTREME_VERBOSE
  cout<<"k="<<k<<";"<<endl;
#endif

  // resize output and initialize with initial guess
  L = L0;
#ifndef FIXED_ITERATIONS_COUNT
  // Keep track of last solution
  Eigen::MatrixXf L_prev;
#endif
  // We will be iterating on L_float, only at the end we convert back to double
  Eigen::MatrixXf L_float = L.cast<float>();

  // Fixed transformation entries also only change at most once per "frame"
  Eigen::MatrixXd Y;
  if(data.fixed_dim.size() == 0)
  {
    Y.resize(0,1);
  }else
  {
    Eigen::Matrix<int,Eigen::Dynamic,1> zero(1);
    zero << 0;
    slice(L0,data.fixed_dim,zero,Y);
#ifdef EXTREME_VERBOSE
    cout<<"Y=["<<endl<<Y<<endl<<"];"<<endl;
#endif
  }

  int iters = 0;
#ifndef FIXED_ITERATIONS_COUNT
  double max_diff = tol+1;  
#endif

  MatrixXf S(k*data.dim,data.dim);
  MatrixXf R(data.dim,data.dim*k);
  Eigen::Matrix<float,Eigen::Dynamic,1> Rcol(data.dim * data.dim * k);
  Matrix<float,Dynamic,1> B_eq_float = B_eq.cast<float>();
  Matrix<float,Dynamic,1> B_eq_fix_float;
  Matrix<float,Dynamic,1> L0float = L0.cast<float>();
  slice(L0float, data.fixed_dim, B_eq_fix_float);	  
  MatrixXf rhsFull(Rcol.rows() + B_eq.rows() + B_eq_fix_float.rows(), 1); 

  MatrixXf Lsep(data.m*(data.dim + 1), 3);  
  const MatrixXf L_part2 = data.M_FullSolve.block(0, Rcol.rows(), data.M_FullSolve.rows(), B_eq_float.rows()) * B_eq_float;
  const MatrixXf L_part3 = data.M_FullSolve.block(0, Rcol.rows() + B_eq_float.rows(), data.M_FullSolve.rows(), B_eq_fix_float.rows()) * B_eq_fix_float;
  MatrixXf L_part2and3 = L_part2 + L_part3;

  // preallocate workspace variables:
  MatrixXf Rxyz(k*data.dim, data.dim);  
  MatrixXf L_part1xyz((data.dim + 1) * data.m, data.dim);
  MatrixXf L_part1(data.dim * (data.dim + 1) * data.m, 1);

#ifdef MATLAB_DEBUG
  mlinit(&g_pEngine);
  mlsetmatrix(&g_pEngine, "CSM_M0", data.CSM_M[0]);
  mlsetmatrix(&g_pEngine, "CSM_M1", data.CSM_M[1]);
  mlsetmatrix(&g_pEngine, "CSM_M2", data.CSM_M[2]);
  mlsetmatrix(&g_pEngine, "FSolve", data.M_FullSolve);  
  MatrixXd M_KGfull = data.M_KG;
  mlsetmatrix(&g_pEngine, "KG", M_KGfull);  

  mlsetmatrix(&g_pEngine, "CSM", CSM);  
#endif

#ifdef ARAP_GLOBAL_TIMING
	  double timer_prepFinished = get_seconds_hires();
#endif

#ifdef FIXED_ITERATIONS_COUNT
  while(iters < max_iters)
#else
  while(iters < max_iters && max_diff > tol)
#endif
  {	
    if(data.print_timings)
    {
      sec_start = get_seconds_hires();
    }

#ifndef FIXED_ITERATIONS_COUNT
    L_prev = L_float;
#endif
    ///////////////////////////////////////////////////////////////////////////
    // Local step: Fix positions, fit rotations
    ///////////////////////////////////////////////////////////////////////////		
	
    // Gather covariance matrices    

    splitColumns(L_float, data.m, data.dim, data.dim + 1, Lsep);
    S = data.CSM * Lsep;//wangyu // interestingly, this doesn't seem to be so slow, but MKL is still 2x faster (probably due to AVX)
    //MKL_matMatMult_single(S, data.CSM, Lsep);//wangyu
    
    if(data.print_timings)
    {
    	sec_covGather = get_seconds_hires();
    }

#ifdef EXTREME_VERBOSE
    cout<<"S=["<<endl<<S<<endl<<"];"<<endl;
#endif
    // Fit rotations to covariance matrices
#ifdef __APPLE__
    if(data.effective_dim == 2)
    {
      fit_rotations_planar(S,R);
    }else
    {
      fit_rotations(S,R);
    }
#else
    fit_rotations_SSE(S,R);		
    //fit_rotations_AVX(S, R);
#endif

#ifdef EXTREME_VERBOSE
    cout<<"R=["<<endl<<R<<endl<<"];"<<endl;
#endif	

    if(data.print_timings)
    {
      sec_fitRotations = get_seconds_hires();
    }
	
    ///////////////////////////////////////////////////////////////////////////
    // "Global" step: fix rotations per mesh vertex, solve for
    // linear transformations at handles
    ///////////////////////////////////////////////////////////////////////////

    // all this shuffling is retarded and not completely negligible time-wise;
    // TODO: change fit_rotations_XXX so it returns R in the format ready for
    // CSolveBlock1 multiplication
    columnize<float,1>(R, k, 2, Rcol);
#ifdef EXTREME_VERBOSE
    cout<<"Rcol=["<<endl<<Rcol<<endl<<"];"<<endl;
#endif	
    splitColumns(Rcol, k, data.dim, data.dim, Rxyz);
    
    if(data.print_timings)
    {
    	sec_prepMult = get_seconds_hires();
    }	
    
    L_part1xyz = data.CSolveBlock1 * Rxyz;//wangyu
    //MKL_matMatMult_single(L_part1xyz, data.CSolveBlock1, Rxyz);	//wangyu	
    mergeColumns(L_part1xyz, data.m, data.dim, data.dim + 1, L_part1);
    
    //L_float = L_part1 + L_part2and3;
    assert(L_float.rows() == L_part1.rows() && L_float.rows() == L_part2and3.rows());
    for (int i=0; i<L_float.rows(); i++) L_float(i, 0) = L_part1(i, 0) + L_part2and3(i, 0);

#ifdef EXTREME_VERBOSE
    cout<<"L=["<<endl<<L<<endl<<"];"<<endl;
#endif	

#ifdef ARAP_DOF_FIX_FLIPS
    // This doesn't work/do what I want it to
    MatrixXd Lstack;
    uncolumnize(L,data.dim,data.dim+1,2,Lstack);
    // loop over transformations
    for(int i = 0;i<data.m;i++)
    {

      // check determinant for flip
      if(Lstack.block(0,i*(data.dim+1),data.dim,data.dim).determinant() < 0)
      {
        // old linear transformation at i
        MatrixXd Li = Lstack.block(0,i*(data.dim+1),data.dim,data.dim);
        // old translation at i
        VectorXd ti = Lstack.block(0,i*(data.dim+1)+data.dim,data.dim,1);
        // Recover frame origin of transformation
        VectorXd oi = (MatrixXd::Identity(data.dim,data.dim) - Li).inverse()*ti;
        // flip last column
        Lstack.col(i*(data.dim+1) + (data.dim-1)) *= -1;
        // New Linear transformation at i
        MatrixXd new_Li = Lstack.block(0,i*(data.dim+1),data.dim,data.dim);
        // New translation at i (same origin)
        VectorXd new_ti = oi - new_Li*oi;
        // fix translation
        Lstack.col(i*(data.dim+1)+data.dim) = new_ti;
      }
    }
    columnize(Lstack,data.m,2,L);

#endif
    if(data.print_timings)
    {
      sec_solve = get_seconds_hires();
    }

#ifndef FIXED_ITERATIONS_COUNT
    // Compute maximum absolute difference with last iteration's solution
    max_diff = (L_float-L_prev).eval().array().abs().matrix().maxCoeff();
#endif
    iters++;	

    if(data.print_timings)
    {
      sec_end = get_seconds_hires();
#ifndef __APPLE__
      printf(
        "\ntotal iteration time = %f "
        "[local: covGather = %f, "
        "fitRotations = %f, "
        "global: prep = %f, "
        "solve = %f, "
        "error = %f [ms]]\n", 
        (sec_end - sec_start)*1000.0, 
      	(sec_covGather - sec_start)*1000.0, 
        (sec_fitRotations - sec_covGather)*1000.0, 
        (sec_prepMult - sec_fitRotations)*1000.0, 
      	(sec_solve - sec_prepMult)*1000.0, 
        (sec_end - sec_solve)*1000.0 );
#endif
    }
  }

  L = L_float.cast<double>();
  assert(L.cols() == 1);

#ifdef ARAP_GLOBAL_TIMING
  double timer_finito = get_seconds_hires();
  printf(
    "ARAP preparation = %f, "
    "all %i iterations = %f [ms]\n", 
    (timer_prepFinished - timer_start)*1000.0, 
    max_iters, 
    (timer_finito - timer_prepFinished)*1000.0);  
#endif

  return true;
}
