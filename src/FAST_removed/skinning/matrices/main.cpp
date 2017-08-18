#include <Eigen/Core>

#define VERBOSE
#include <cotmatrix.h>
#include <read.h>
#include <print_ijv.h>
//#include <adjacency_matrix.h>
//#include <sum.h>
//#include <diag.h>
//#include <edges.h>
#include <readDMAT.h>
//#include <mat_max.h>
//#include <partition.h>
//#include <group_sum_matrix.h>
//#include <repmat.h>
//#include <repdiag.h>
//#include <sparse.h>
//#include <slice.h>
//#include <sort.h>
//#include <speye.h>
//#include <colon.h>
//#include <min_quad_with_fixed.h>
//#include <full.h>
//#include <cotangent.h>
#include <readMESH.h>
//#include <cat.h>
#include <upsample.h>
#include <limit_faces.h>
#include <faces_first.h>


#include <iostream>
#include <functional>


#include "lbs_matrix.h"
#include "covariance_scatter_matrix.h"
#include "fit_rotations.h"
#include "arap_rhs.h"
#include "columnize.h"
#include "uncolumnize.h"
#include "arap_dof.h"
#include "arap_linear_block.h"
#include "Ease.h"
#include "iso_extra.h"
#include "uniform_sample.h"

using namespace igl;
using namespace std;
using namespace Eigen;

int main(int argc,char * argv[])
{
  if(argc <=2)
  {
    printf("USAGE:\n  ./matrices [mesh(.obj|.mesh)] [weights.dmat]\n");
    return 1;
  }

  // Load mesh
  MatrixXd V;
  MatrixXi F;
  MatrixXi T;
  if(!read(argv[1],V,F))
  {
    if(!readMESH(argv[1],V,T,F))
    {
      return 1;
    }
  }

  // Load weights matrix
  MatrixXd OW;
  if(!readDMAT(argv[2],OW))
  {
    return 1;
  }

  //// Only keep x and y
  //V = V.block(0,0,V.rows(),2);

  cout<<"V=["<<endl<<V<<endl<<"];"<<endl;
  cout<<"F=["<<endl<<F<<endl<<"]+1;"<<endl;
  if(T.size() != 0)
  {
    cout<<"T=["<<endl<<T<<endl<<"]+1;"<<endl;
  }
  cout<<"OW=["<<endl<<OW<<endl<<"];"<<endl;
  MatrixXi Ele = (T.size() == 0 ? F : T);
#ifdef EXTRA

  // lbs matrix
  SparseMatrix<double> M;
  lbs_matrix(V,OW,M);
  cout<<"MIJV=["<<endl;print_ijv(M,1);cout<<endl<<"];"<<
    endl<<"M=sparse(MIJV(:,1),MIJV(:,2),MIJV(:,3),"<<
    M.rows()<<","<<M.cols()<<");"<<endl;

  MatrixXd Mfull;
  full(M,Mfull);
  cout<<"Mfull=["<<Mfull<<"];"<<endl;
  
  int k = 3;
  cout<<"k="<<k<<";"<<endl;
  Matrix<int,Dynamic,1> G;
  Matrix<int,Dynamic,1> S;
  Matrix<double,Dynamic,1> GD;
  partition(OW,k,G,S,GD);
  cout<<"S=["<<endl<<S<<endl<<"]+1;"<<endl;
  cout<<"GD=["<<endl<<GD<<endl<<"];"<<endl;
  cout<<"G=["<<endl<<G<<endl<<"]+1;"<<endl;

  Matrix<int,Dynamic,1> free(0,1);
  cout<<"free=["<<endl<<free<<endl<<"]+1;"<<endl;

  Matrix<int,Dynamic,1> fixed(0,1);
  cout<<"fixed=["<<endl<<fixed<<endl<<"]+1;"<<endl;
  
  MatrixXd C(2,V.cols());
  C.row(0) = V.row(0);
  C.row(1) = V.row(8);
  C(0,0) += 1;
  C(1,0) += 1;
  cout<<"C=["<<endl<<C<<endl<<"];"<<endl;

  arap_dof_data data;
  arap_dof_precomputation(V,F,M,G,free,fixed,C,data);

  // Dummy initial guess of identities
  int dim = V.cols();
  int m = C.rows();
  MatrixXd I = MatrixXd::Identity(dim,dim+1);
  cout<<"I=["<<endl<<I<<endl<<"];"<<endl;
  MatrixXd Istack;
  repmat(I,1,m,Istack);
  cout<<"Istack=["<<endl<<Istack<<endl<<"];"<<endl;
  MatrixXd L0;
  columnize(Istack,m,2,L0);
  cout<<"L0=["<<endl<<L0<<endl<<"];"<<endl;

  // Dummy displacements
  MatrixXd D = MatrixXd::Zero(m,dim);
  D(0,0) = 100;
  cout<<"D=["<<endl<<D<<endl<<"];"<<endl;

  MatrixXd L;
  arap_dof_update(data,D,L0,1,0,L);
  cout<<"L=["<<endl<<L<<endl<<"];"<<endl;

  MatrixXd Lstack;
  uncolumnize(L,dim,dim+1,2,Lstack);
  cout<<"Lstack=["<<endl<<Lstack<<endl<<"];"<<endl;

  SparseMatrix<double> L;
  cotmatrix(V,F,L);
  cout<<"LIJV=["<<endl;print_ijv(L,1);cout<<endl<<"];"<<
    endl<<"L=sparse(LIJV(:,1),LIJV(:,2),LIJV(:,3),"<<
    L.rows()<<","<<L.cols()<<");"<<endl;

  SparseMatrix<double> A;
  adjacency_matrix(F,A);
  cout<<"AIJV=["<<endl;print_ijv(A,1);cout<<endl<<"];"<<endl;

  SparseVector<double> Asum;
  sum(A,1,Asum);
  Matrix<int,Eigen::Dynamic,1> AsumI;
  Matrix<double,Eigen::Dynamic,1> AsumV;
  find(Asum,AsumI,AsumV);
  cout<<"Asum=["<<endl<<AsumV<<endl<<"];"<<endl;


  SparseMatrix<double> Adiag;
  diag(Asum,Adiag);
  cout<<"AdiagIJV=["<<endl;print_ijv(Adiag,1);cout<<endl<<"];"<<endl;

  SparseVector<double> Adiagdiag;
  diag(Adiag,Adiagdiag);
  Matrix<int,Eigen::Dynamic,1> AdiagdiagI;
  Matrix<double,Eigen::Dynamic,1> AdiagdiagV;
  find(Adiagdiag,AdiagdiagI,AdiagdiagV);
  cout<<"Adiagdiag=["<<endl<<AdiagdiagV<<endl<<"];"<<endl;

  // Build uniform laplacian
  SparseMatrix<double> U;
  U = A-Adiag;
  cout<<"UIJV=["<<endl;print_ijv(U,1);cout<<endl<<"];"<<endl;

  Eigen::MatrixXi E;
  edges(F,E);
  cout<<"E=["<<endl<<E<<endl<<"]+1;"<<endl;

  // Lbs matrix
  //Eigen::MatrixXd M;
  SparseMatrix<double> M;
  lbs_matrix(V,OW,M);
  cout<<"MIJV=["<<endl;print_ijv(M,1);cout<<endl<<"];"<<endl;

  const int MAX_NUMBER_OF_WEIGHTS_PER_VERTEX = OW.cols();
  // Sort weights and indices for each mesh vertex (row) descending
  const int sort_dim = 2;
  const bool ascending = false;
  MatrixXd W;
  MatrixXi WI;
  sort(OW,sort_dim,ascending,W,WI);
  // Clip of least significant weights if there are more than supported by the
  // shader
  if(W.cols() > MAX_NUMBER_OF_WEIGHTS_PER_VERTEX)
  {
    verbose(
      "WARNING: too many weights (%d) per mesh vertex."
      " Keeping only %d most significant.\n",
      W.cols(),
      MAX_NUMBER_OF_WEIGHTS_PER_VERTEX);
    W = W.leftCols(MAX_NUMBER_OF_WEIGHTS_PER_VERTEX);
    WI = WI.leftCols(MAX_NUMBER_OF_WEIGHTS_PER_VERTEX);
  }
  SparseMatrix<double> M2;
  lbs_matrix(V,W,WI,M2);
  cout<<"M2IJV=["<<endl;print_ijv(M2,1);cout<<endl<<"];"<<endl;


  // Covariance scatter matrix
  SparseMatrix<double> CSM;
  covariance_scatter_matrix(V,F,CSM);
  cout<<"CSMIJV=["<<endl;print_ijv(CSM,1);cout<<endl<<"];"<<
    endl<<"CSM=sparse(CSMIJV(:,1),CSMIJV(:,2),CSMIJV(:,3),"<<
    CSM.rows()<<","<<CSM.cols()<<");"<<endl;

  Matrix<double,Dynamic,1> Y;
  Matrix<int,Dynamic,1> I;
  mat_max(V,2,Y,I);
  cout<<"Y=["<<endl<<Y<<endl<<"];"<<endl;
  cout<<"I=["<<endl<<I<<endl<<"]+1;"<<endl;

  int k = 3;
  cout<<"k="<<k<<";"<<endl;
  Matrix<int,Dynamic,1> G;
  Matrix<int,Dynamic,1> S;
  Matrix<double,Dynamic,1> D;
  partition(OW,k,G,S,D);
  cout<<"S=["<<endl<<S<<endl<<"]+1;"<<endl;
  cout<<"D=["<<endl<<D<<endl<<"];"<<endl;
  cout<<"G=["<<endl<<G<<endl<<"]+1;"<<endl;

  SparseMatrix<double> G_sum;
  group_sum_matrix(G,k,G_sum);
  cout<<"G_sumIJV=["<<endl;print_ijv(G_sum,1);cout<<endl<<"];"<<
    endl<<"G_sum=sparse(G_sumIJV(:,1),G_sumIJV(:,2),G_sumIJV(:,3),"<<
    G_sum.rows()<<","<<G_sum.cols()<<");"<<endl;


  MatrixXd VV;
  repmat(V,2,3,VV);
  cout<<"VV=["<<endl<<VV<<endl<<"];"<<endl;

  MatrixXd R;
  fit_rotations(V,F,V,CSM,R);
  cout<<"R=["<<endl<<R<<endl<<"];"<<endl;

  MatrixXd V2;
  V2.resize(V.rows(),2);
  verbose("%d %d\n",V.rows(),V.cols());
  V2.col(0) = V.col(0);
  V2.col(1) = V.col(1);
  verbose("%d %d\n",V.rows(),V.cols());
  Eigen::SparseMatrix<double> K;
  arap_rhs(V2,F,K);
  cout<<"KIJV=["<<endl;print_ijv(K,1);cout<<endl<<"];"<<
    endl<<"K=sparse(KIJV(:,1),KIJV(:,2),KIJV(:,3),"<<
    K.rows()<<","<<K.cols()<<");"<<endl;


  MatrixXd V3;
  repdiag(V,3,V3);
  cout<<"V3=["<<endl<<V3<<endl<<"];"<<endl;

  SparseMatrix<double> L5;
  repdiag(L,5,L5);
  cout<<"L5IJV=["<<endl;print_ijv(L5,1);cout<<endl<<"];"<<
    endl<<"L5=sparse(L5IJV(:,1),L5IJV(:,2),L5IJV(:,3),"<<
    L5.rows()<<","<<L5.cols()<<");"<<endl;

  MatrixXd B;
  B.resize(2,3);
  B<<1,2,3,4,5,6;
  cout<<"B=["<<endl<<B<<endl<<"];"<<endl;

  MatrixXd HB;
  repmat(B,1,10,HB);
  cout<<"HB=["<<endl<<HB<<endl<<"];"<<endl;
  Matrix<double,Eigen::Dynamic,1> HBcol;
  columnize(HB,10,2,HBcol);
  cout<<"HBcol=["<<endl<<HBcol<<endl<<"];"<<endl;

  MatrixXd VB;
  repmat(B,10,1,VB);
  cout<<"VB=["<<endl<<VB<<endl<<"];"<<endl;
  Matrix<double,Eigen::Dynamic,1> VBcol;
  columnize(VB,10,1,VBcol);
  cout<<"VBcol=["<<endl<<VBcol<<endl<<"];"<<endl;

  Matrix<int,Eigen::Dynamic,1> LI;
  Matrix<int,Eigen::Dynamic,1> LJ;
  Matrix<double,Eigen::Dynamic,1> LV;
  find(L,LI,LJ,LV);
  SparseMatrix<double> Lre;
  sparse(LI,LJ,LV,Lre);
  cout<<"LreIJV=["<<endl;print_ijv(Lre,1);cout<<endl<<"];"<<
    endl<<"Lre=sparse(LreIJV(:,1),LreIJV(:,2),LreIJV(:,3),"<<
    Lre.rows()<<","<<Lre.cols()<<");"<<endl;

  SparseMatrix<double> Z;
  Z = L-L.transpose();
  cout<<"nnz = "<<Z.nonZeros()<<";"<<endl;
  cout<<"ZIJV=["<<endl;print_ijv(Z,1);cout<<endl<<"];"<<
    endl<<"Z=sparse(ZIJV(:,1),ZIJV(:,2),ZIJV(:,3),"<<
    Z.rows()<<","<<Z.cols()<<");"<<endl;

  SparseMatrix<double> I;
  speye(2,4,I);
  cout<<"IIJV=["<<endl;print_ijv(I,1);cout<<endl<<"];"<<
    endl<<"I=sparse(IIJV(:,1),IIJV(:,2),IIJV(:,3),"<<
    I.rows()<<","<<I.cols()<<");"<<endl;
  speye(4,2,I);
  cout<<"IIJV=["<<endl;print_ijv(I,1);cout<<endl<<"];"<<
    endl<<"I=sparse(IIJV(:,1),IIJV(:,2),IIJV(:,3),"<<
    I.rows()<<","<<I.cols()<<");"<<endl;
  speye(4,I);
  cout<<"IIJV=["<<endl;print_ijv(I,1);cout<<endl<<"];"<<
    endl<<"I=sparse(IIJV(:,1),IIJV(:,2),IIJV(:,3),"<<
    I.rows()<<","<<I.cols()<<");"<<endl;

  Eigen::Matrix<double,Eigen::Dynamic,1> count;
  colon(1,4,count);
  cout<<"count=["<<endl<<count<<endl<<"];"<<endl;
  colon(4,-1,1,count);
  cout<<"count=["<<endl<<count<<endl<<"];"<<endl;
  colon(4,-0.5,1,count);
  cout<<"count=["<<endl<<count<<endl<<"];"<<endl;
  colon(4,-3,1,count);
  cout<<"count=["<<endl<<count<<endl<<"];"<<endl;
  count = colon<double>(1,10);
  cout<<"count=["<<endl<<count<<endl<<"];"<<endl;

  SparseMatrix<double> LL;
  repmat(L,2,3,LL);
  cout<<"LLIJV=["<<endl;print_ijv(LL,1);cout<<endl<<"];"<<
    endl<<"LL=sparse(LLIJV(:,1),LLIJV(:,2),LLIJV(:,3),"<<
    LL.rows()<<","<<LL.cols()<<");"<<endl;

  Matrix<int,Eigen::Dynamic,1> known;
  known.resize(2);
  known<<0,V.rows()-1;
  cout<<"known=["<<endl<<known<<endl<<"]+1;"<<endl;
  MatrixXd Y = MatrixXd::Identity(known.size(),known.size());
  cout<<"Y=["<<endl<<Y<<endl<<"];"<<endl;


  Matrix<int,Eigen::Dynamic,1> unknown;
  unknown.resize(7);
  unknown<<1,2,3,4,5,6,7;
  cout<<"unknown=["<<endl<<unknown<<endl<<"]+1;"<<endl;

  SparseMatrix<double> Lku;
  slice(L,known,unknown,Lku);
  cout<<"LkuIJV=["<<endl;print_ijv(Lku,1);cout<<endl<<"];"<<
    endl<<"Lku=sparse(LkuIJV(:,1),LkuIJV(:,2),LkuIJV(:,3),"<<
    Lku.rows()<<","<<Lku.cols()<<");"<<endl;


  SparseMatrix<double> A = L*-1.0;
  cout<<"AIJV=["<<endl;print_ijv(A,1);cout<<endl<<"];"<<
    endl<<"A=sparse(AIJV(:,1),AIJV(:,2),AIJV(:,3),"<<
    A.rows()<<","<<A.cols()<<");"<<endl;
  Matrix<double,Dynamic,1> B = Matrix<double,Dynamic,1>::Zero(V.rows());
  cout<<"B=["<<endl<<B<<endl<<"];"<<endl;
  SparseMatrix<double> Aeq(0,0);
  cout<<"AeqIJV=["<<endl;print_ijv(Aeq,1);cout<<endl<<"];"<<
    endl<<"Aeq=sparse(AeqIJV(:,1),AeqIJV(:,2),AeqIJV(:,3),"<<
    Aeq.rows()<<","<<Aeq.cols()<<");"<<endl;
  Matrix<double,Dynamic,1> Beq = Matrix<double,Dynamic,1>::Zero(0);
  cout<<"Beq=["<<endl<<Beq<<endl<<"];"<<endl;

  min_quad_with_fixed_data<double> data;
  min_quad_with_fixed_precompute(
    A,
    known,
    Aeq,
    true,
    data);

  Matrix<double,Dynamic,Dynamic> Z;
  min_quad_with_fixed_solve(
    data,
    B,
    Y,
    Beq,
    Z);
  cout<<"Z=["<<endl<<Z<<endl<<"];"<<endl;

  Aeq.resize(1,V.rows());
  Aeq.reserve(2);
  Aeq.insert(0,1) = 1;
  Aeq.insert(0,2) = -1;
  Aeq.finalize();
  cout<<"AeqIJV=["<<endl;print_ijv(Aeq,1);cout<<endl<<"];"<<
    endl<<"Aeq=sparse(AeqIJV(:,1),AeqIJV(:,2),AeqIJV(:,3),"<<
    Aeq.rows()<<","<<Aeq.cols()<<");"<<endl;
  Beq.resize(1,1);
  Beq(0) = 0;
  cout<<"Beq=["<<endl<<Beq<<endl<<"];"<<endl;

  bool precompute_success = 
    min_quad_with_fixed_precompute(
      A,
      known,
      Aeq,
      true,
      data);
  assert(precompute_success);

  min_quad_with_fixed_solve(
    data,
    B,
    Y,
    Beq,
    Z);
  cout<<"Z=["<<endl<<Z<<endl<<"];"<<endl;

 MatrixXd C;
  cotangent(V,Ele,C);
  cout<<"C=["<<endl<<C<<endl<<"];"<<endl;

  ArapEnergy energy = ARAP_SPOKES;

  SparseMatrix<double> Kd;
  arap_linear_block(V,Ele,1,energy,Kd);
  cout<<"KdIJV=["<<endl;print_ijv(Kd,1);cout<<endl<<"];"<<
    endl<<"Kd=sparse(KdIJV(:,1),KdIJV(:,2),KdIJV(:,3),"<<
    Kd.rows()<<","<<Kd.cols()<<");"<<endl;

  SparseMatrix<double> CSM;
  covariance_scatter_matrix(V,Ele,energy,CSM);
  cout<<"CSMIJV=["<<endl;print_ijv(CSM,1);cout<<endl<<"];"<<
    endl<<"CSM=sparse(CSMIJV(:,1),CSMIJV(:,2),CSMIJV(:,3),"<<
    CSM.rows()<<","<<CSM.cols()<<");"<<endl;

  SparseMatrix<double> K;
  arap_rhs(V,Ele,energy,K);
  cout<<"KIJV=["<<endl;print_ijv(K,1);cout<<endl<<"];"<<
    endl<<"K=sparse(KIJV(:,1),KIJV(:,2),KIJV(:,3),"<<
    K.rows()<<","<<K.cols()<<");"<<endl;

  SparseMatrix<double> L_opt;
  cotmatrix(V,Ele,L_opt);
  cout<<"L_optIJV=["<<endl;print_ijv(L_opt,1);cout<<endl<<"];"<<
    endl<<"L_opt=sparse(L_optIJV(:,1),L_optIJV(:,2),L_optIJV(:,3),"<<
    L_opt.rows()<<","<<L_opt.cols()<<");"<<endl;
#endif

  //MatrixXd EW;
  //iso_extra(OW,3,2,EASE_CUBIC,EW);
  //cout<<"EW=["<<endl<<EW<<endl<<"];"<<endl;

  //MatrixXd VV;
  //MatrixXi FF;
  //upsample(V,F,VV,FF);
  //cout<<"VV=["<<endl<<VV<<endl<<"];"<<endl;
  //cout<<"FF=["<<endl<<FF<<endl<<"]+1;"<<endl;

  //MatrixXd OWOW = OW;
  //MatrixXi FF = F;
  //for(int i = 0;i<8;i++)
  //{
  //  //cout<<"OWOW=["<<endl<<OWOW<<endl<<"];"<<endl;
  //  //cout<<"FF=["<<endl<<FF<<endl<<"]+1;"<<endl;
  //  verbose("FF.rows(): %d\n",FF.rows());

  //  MatrixXd WS;
  //  uniform_sample(OWOW,FF,2,2,WS);
  //  cout<<"WS=["<<endl<<WS<<endl<<"];"<<endl;
  //  upsample(OWOW,FF);
  //}

  MatrixXd WS;
  uniform_sample(OW,T,2,2,WS);
  cout<<"WS=["<<endl<<WS<<endl<<"];"<<endl;

  //VectorXi L(3);
  //L << 0,3,1;
  //MatrixXi LF;
  //limit_faces(F,L,true,LF);
  //cout<<"LF=["<<endl<<LF<<endl<<"];"<<endl;

  //MatrixXi FF(2,3);
  //FF << 0,3,1,7,8,5;
  //MatrixXd RV;
  //MatrixXi RF;
  //VectorXi IM;
  //faces_first(V,FF,RV,RF,IM);
  //cout<<"RV=["<<endl<<RV<<endl<<"];"<<endl;
  //cout<<"RF=["<<endl<<RF<<endl<<"];"<<endl;

  return 0;
}
